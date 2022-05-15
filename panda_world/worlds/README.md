### Empty world with the plugin libgazebo_ros_link_attacher.so loaded (in the worlds folder).

Clone the dependent package in your workspace/src ```git clone git@github.com:pal-robotics/gazebo_ros_link_attacher.git``` 
Run ```catkin_make``` 
Add the gazebo plugin in world file 

```
  <world name='default'>
    <plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>
```

