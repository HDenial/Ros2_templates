from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='HugoF',
    maintainer_email='HugoF@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main", # Entry point for the console script, exec name(py_node) = pkg_name(my_py_pkg).node_file(my_first_node):node_function_name(main)
            "robot_publisher = my_py_pkg.robot_publisher:main",
            "robot_listener = my_py_pkg.robot_listener:main",
            "number = my_py_pkg.number_param:main",
            "counter = my_py_pkg.counter:main",
            "sum_service = my_py_pkg.sum_service:main", 
            "sum_client = my_py_pkg.sum_client:main",
            "hardware_status = my_py_pkg.hardware_status:main",
            "battery_state = my_py_pkg.battery_state:main",
            "led_panel = my_py_pkg.led_panel:main",
            "rpi_led = my_py_pkg.rpi_led:main",
            "rpi_manager = my_py_pkg.rpi_keyman:main",
        ],
    },
)
# DO NOT FORGET TO INSERT COMMA AFTER EVERY ENTRY IN THE LIST
# to run the node, use the command:
# ros2 run my_py_pkg py_node on ~ directory
# to build the package, use the command:
# colcon build  
# to source the package, add the command:
# source install/setup.bash to your ~/.bashrc file
# or run it directly in the terminal before running the node after building the package