from setuptools import setup

package_name = 'mini_pupper_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='MangDang',
    author_email='fae@mangdang.net',
    maintainer='MangDang',
    maintainer_email='fae@mangdang.net',
    description='The Mini Pupper IMU driver package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mini_pupper_imu = mini_pupper_imu.mini_pupper_imu:main'
        ],
    },
)
