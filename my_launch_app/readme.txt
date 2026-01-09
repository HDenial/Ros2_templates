//basic launch file structure

<launch>
    <node pkg='<pkg_name>' exec='exec_name' name='<rename>'/>

    <node pkg='<pkg_name>' exec='exec_name' name='<rename>'>
      <remap from="/<og_topic_name>" to="/<new_topic_name>" />
      <param name="<param_name>" type="<value type>" value="<new_value>" />
      or if you have a yaml config file:
      <param from="$(find-pkg-share <pkg_name>)/<location>/<config_name>.yaml" />
    </node>
</launch>

integration of launches:
<include file="$(find-pkg-share <pkg_name>)/<location>/<launch_name>.*" />

//executable names are on the setup.* file of the given pkg
//add to package.xml exec_depend <pkg_name>

//add these lines to CMakelists after dependencies:

install ( DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
