from setuptools import find_packages, setup

package_name = 'main_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elf',
    maintainer_email='elf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_controller = main_controller.UAVController:main',
            'position_checker_ros2 = main_controller.position_checker_ros2:main',
            'demo1 = main_controller.demo1:main',
            'data_procure = main_controller.data_procure:main',
            'take_picture = main_controller.take_picture:main',
        ],
    },
)
