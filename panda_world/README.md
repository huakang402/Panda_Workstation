# panda_world

## package decription：

​	Panda workstation spawn.

## How to use

- Run ```roslaunch panda_world gazebo_spawn_panda.launch```.  
- Click start simulation in Gazebo and ``save world as`` ``panda_workstation`` in ``worlds`` directory.(If you want to save as other name, you must ensure that the ``world_name`` is consistent with it in ``panda_moveit_config/launch/gazebo.launch``.)  
- Run ```roslaunch panda_world panda_gazebo.launch``` to load panda workstation.  

