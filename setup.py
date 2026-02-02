from setuptools import find_packages, setup

package_name = 'jr3_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartek Łukawski',
    maintainer_email='bwmn.peter@gmail.com',
    description='Driver for JR3 force-torque sensor',
    license='LGPL-v2.1',
    entry_points={
        'console_scripts': [
            'jr3_driver = jr3_driver.jr3_driver:main'
        ],
    },
)
