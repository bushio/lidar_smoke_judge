from setuptools import setup
import glob
import os

package_name = 'lidar_smoke_judge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*xml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bushio',
    maintainer_email='satoshi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_smoke_judge_node = lidar_smoke_judge.lidar_smoke_judge:main',

        ],
    },
)
