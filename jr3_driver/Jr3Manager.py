import serial
import threading
import time

from enum import Enum
from queue import Queue, Empty

TIMEOUT_READ = 2.0 # seconds

class Jr3Command(Enum):
    ACK = 1
    START = 2
    STOP = 3
    ZERO_OFFS = 4
    SET_FILTER = 5
    GET_STATE = 6
    GET_FS = 7
    RESET = 8
    READ = 9
    BOOTUP = 10

class Jr3State(Enum):
    READY = 0
    NOT_INITIALIZED = 1

class SerialMsg:
    def __init__(self, op: int, data: bytes = bytes()):
        self.op = op
        self.data = data
        self.size = len(self.data)

class Jr3Manager:
    def __init__(self, channel: str, baudrate: int, timeout: float = TIMEOUT_READ):
        self._ser = serial.Serial(channel, baudrate)
        self._acknowledged_msgs_queue = Queue(maxsize=1)
        self._fs_factors = None
        self._last_state_time = None
        self._state = Jr3State.NOT_INITIALIZED
        self._forces = None
        self._torques = None
        self._last_ft_time = None
        self._framecounter = 0
        self._timeout_read = timeout
        self._running = True
        self._thread = threading.Thread(target=self._read_worker)
        self._thread.start()

    def __del__(self):
        self.close()

    def close(self) -> None:
        self._running = False
        self._thread.join()
        self._send_message(SerialMsg(Jr3Command.STOP.value))
        self._ser.close()

    def start(self, cutoff_freq: float, period_ms: float) -> tuple[bool, Jr3State]:
        if not self.get_fs()[0]:
            return False, self._state

        cutoff_freq = int(cutoff_freq * 100) # [0.01 Hz]
        period_ms = int(period_ms * 1000) # [us]

        data = cutoff_freq.to_bytes(2, 'little') + period_ms.to_bytes(4, 'little')
        msg = SerialMsg(Jr3Command.START.value, data)
        success = self._send_ack_command(msg)
        return success, self._state

    def stop(self) -> tuple[bool, Jr3State]:
        msg = SerialMsg(Jr3Command.STOP.value)
        success = self._send_ack_command(msg)
        return success, self._state

    def zero_offs(self) -> tuple[bool, Jr3State]:
        msg = SerialMsg(Jr3Command.ZERO_OFFS.value)
        success = self._send_ack_command(msg)
        return success, self._state

    def set_filter(self, cutoff_freq: int) -> tuple[bool, Jr3State]:
        msg = SerialMsg(Jr3Command.SET_FILTER.value, cutoff_freq.to_bytes(2, 'little'))
        success = self._send_ack_command(msg)
        return success, self._state

    def get_state(self) -> tuple[bool, Jr3State]:
        if self._state is None or self._last_state_time is None or (time.time() - self._last_state_time > self._timeout_read):
            msg = SerialMsg(Jr3Command.GET_STATE.value)

            if not self._send_ack_command(msg):
                return False, self._state

        return True, self._state

    def get_fs(self) -> tuple[bool, (list[int] | None), Jr3State]:
        if self._fs_factors is None:
            msg_out = SerialMsg(Jr3Command.GET_FS.value)
            success = self._send_ack_command(msg_out, lambda msg_in: self._populate_fs_factors(msg_in.data))

            if not success or self._fs_factors is None or len(self._fs_factors) != 6:
                return False, None, self._state

        return True, self._fs_factors, self._state

    def reset(self) -> tuple[bool, Jr3State]:
        msg = SerialMsg(Jr3Command.RESET.value)
        success = self._send_ack_command(msg)
        return success, self._state

    def read(self) -> tuple[bool, (list[float] | None), (list[float] | None), int]:
        if not self._last_ft_time is None and time.time() - self._last_ft_time < self._timeout_read / 10:
            return True, self._forces, self._torques, self._framecounter

        return False, None, None, 0

    def _read_worker(self) -> None:
        while self._running:
            self._read_message()
            time.sleep(0.001)

    def _send_ack_command(self, msg_out: SerialMsg, callback = None) -> bool:
        try:
            # clear previous ACK message
            self._acknowledged_msgs_queue.get(block=False)
        except Empty:
            pass

        if not self._send_message(msg_out):
            return False

        start_time = time.time()

        while time.time() - start_time < self._timeout_read:
            try:
                msg_in = self._acknowledged_msgs_queue.get(timeout=0.01)
                self._state = Jr3State(msg_in.data[0])
                self._last_state_time = start_time

                if callback is not None:
                    callback(msg_in)

                return True
            except Empty:
                continue

        return False

    @staticmethod
    def _build_message(msg: SerialMsg) -> bytearray:
        buffer = bytearray()
        buffer.extend(b'<%02d' % msg.op)

        if msg.size > 0:
            buffer.extend(msg.data)

        buffer.extend(b'>')
        return buffer

    def _send_message(self, msg: SerialMsg) -> bool:
        try:
            message = Jr3Manager._build_message(msg)
            self._ser.write(message)
            return True
        except serial.SerialException as e:
            return False

    def _populate_fs_factors(self, data) -> None:
        if len(data) >= 12:
            self._fs_factors = [int.from_bytes(data[i:i+2], 'little') for i in range(1, len(data), 2)]

    def _parse_ft_message(self, msg: SerialMsg) -> None:
        if msg.size == 14 and self._fs_factors is not None and len(self._fs_factors) == 6:
            self._forces = [2 * int.from_bytes(msg.data[2*i:2*i+2], 'little', signed=True) / self._fs_factors[i] for i in range(0, 3)]
            self._torques = [2 * int.from_bytes(msg.data[2*i:2*i+2], 'little', signed=True) / (self._fs_factors[i] * 100) for i in range(3, 6)]
            self._framecounter = msg.data[6]
            self._last_ft_time = time.time()

    def _read_message(self) -> None:
        buffer_in = self._ser.read_until(b'>')

        for buffer in Jr3Manager._extract_messages(buffer_in):
            try:
                msg = SerialMsg(int(buffer[:2].decode()), buffer[2:])

                if msg.op == Jr3Command.ACK.value and msg.size > 0:
                    self._acknowledged_msgs_queue.put(msg)
                elif msg.op == Jr3Command.READ.value:
                    self._parse_ft_message(msg)
            except Exception as e:
                continue

    @staticmethod
    def _extract_messages(buffer: bytes) -> list[bytes]:
        out = []
        start_idx = buffer.find(b'<')

        while start_idx != -1:
            end_idx = buffer.find(b'>', start_idx)

            if end_idx != -1:
                out.append(buffer[start_idx + 1:end_idx])
                start_idx = buffer.find(b'<', end_idx)
            else:
                break

        return out
