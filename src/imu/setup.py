from setuptools import setup

package_name = 'imu'

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
    maintainer='cidi',
    maintainer_email='1113662441@qq.com',
    description='IMU data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = imu.imu_node:main',
            'pointcloud_node = imu.pointcloud_node:main',
            'imu_fliter = imu.imu_fliter_node:main'
        ],
    },
)
