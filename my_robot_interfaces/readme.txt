When creating a new srv or msg add it to cmakelists 
in this directory.

add to cmakelists.txt:

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"{folder}/{interface_name}.{interface_type}"
""
)

ament_export_dependencies(rosidl_default_runtime)




When using a new custom interface dont forget to add 
it to the CMakeLists.txt file generate rosidl

For cpp files only:
add filepath [home/HugoF/ros2_ws/install/<interface_package>/include/**]
to c_cpp_properties.json under include path
and add  <interface_package> to cmakelists depencies