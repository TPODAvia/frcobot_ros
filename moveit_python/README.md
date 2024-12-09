# moveit python

Please the the tutorial provided in the `docs` folder.

1. Use executor example
```bash
rosrun moveit_python task_executer.py
```

2. Use generator example
```bash
rosrun moveit_python task_generator.py help
rosrun moveit_python task_generator.py robot get_robot_param
rosrun moveit_python task_generator.py fr10 joints_position
rosrun moveit_python task_generator.py fr10 joints_position 0 0 0 0 0 0
rosrun moveit_python task_generator.py fr10 end_coordinate tf_end
rosrun moveit_python task_generator.py fr10 end_coordinate hello_box
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 clear_scene
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect
rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN
rosrun moveit_python task_generator.py fr10 choose_follow_mode
rosrun moveit_python task_generator.py fr10 check_json_files
rosrun moveit_python task_generator.py fr10 delete_json_sim_content test.json
rosrun moveit_python task_generator.py fr10 delete_json_temp
```

Pick and place example for fr10:
```bash
0 `rosrun frcobot_hw frcobot_clear_error`
0 `rosrun moveit_python task_generator.py fr10 clear_scene`
1 `rosrun moveit_python task_generator.py fr10 remove_object hello_box`
2 `rosrun moveit_python task_generator.py fr10 joints_position 0 -1.57 1.57 0 0 0`
3 `rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924`
4 `rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect`
5 `rosrun moveit_python task_generator.py fr10 end_coordinate hello_box`
6 `rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end`
7 `rosrun moveit_python task_generator.py fr10 gripper_close`
8 `rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN`
9 `rosrun moveit_python task_generator.py fr10 end_coordinate tf_end -0.33 0.63 0.11 -0.01 0.709 -0.01 0.705`
10 `rosrun moveit_python task_generator.py fr10 gripper_open`
11 `rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end`
12 `rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect`
13 `rosrun moveit_python task_generator.py fr10 joints_position 0 -1.57 1.57 0 0 0`
14 `rosrun moveit_python task_generator.py fr10 delete_json_sim_content test.json`
```
roslaunch depthai_examples stereo_node.launch
1075596302

Depth camera test 
rosrun moveit_python task_generator.py fr10 joints_position 1.853 -1.989 -2.448 1.400 0.501 2.578
rosrun moveit_python task_generator.py fr10 joints_position 1.761 -1.206 -2.520 0.291 0.845 -2.134
rosrun moveit_python task_generator.py fr10 joints_position 1.232 -1.717 -2.501 1.011 1.530 -0.582
rosrun moveit_python task_generator.py fr10 joints_position 0.744 -1.385 -2.183 -2.465 -2.204 -1.263
rosrun moveit_python task_generator.py fr10 joints_position 2.660 -1.527 2.150 1.257 -0.198 -2.349


rosrun moveit_python task_generator.py fr10 joints_position 2.223 -1.489 1.848 0.139 -0.407 -0.899
rosrun moveit_python task_generator.py fr10 joints_position 1.181 -1.989 -2.079 1.117 1.407 2.678
rosrun moveit_python task_generator.py fr10 joints_position 0.577 -1.613 -2.115 -2.259 -2.361 -1.208
rosrun moveit_python task_generator.py fr10 joints_position 1.534 -2.058 -2.419 1.272 1.211 -0.559
rosrun moveit_python task_generator.py fr10 joints_position 2.367 -1.724 -2.508 0.376 0.302 -1.635
rosrun moveit_python task_generator.py fr10 joints_position 1.554 -2.141 -2.229 1.294 0.816 2.611


/camera/depth/image_raw
/camera/pointcloud/raw
/camera/rgb/camera_info
/camera/rgb/image_raw

Pick and place example for fr3:
```bash
0 `rosrun moveit_python task_generator.py fr3 clear_scene`
1 `rosrun moveit_python task_generator.py fr3 remove_object hello_box`
2 `rosrun moveit_python task_generator.py fr3 joints_position 0 -1.57 1.57 0 0 0`
3 `rosrun moveit_python task_generator.py fr3 spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924`
4 `rosrun moveit_python task_generator.py fr3 choose_pipeline OMPL RRTConnect`
5 `rosrun moveit_python task_generator.py fr3 end_coordinate hello_box`
6 `rosrun moveit_python task_generator.py fr3 attach_object hello_box tf_end`
7 `rosrun moveit_python task_generator.py fr3 gripper_close`
8 `rosrun moveit_python task_generator.py fr3 choose_pipeline PILZ LIN`
9 `rosrun moveit_python task_generator.py fr3 end_coordinate tf_end`
10 `rosrun moveit_python task_generator.py fr3 gripper_open`
11 `rosrun moveit_python task_generator.py fr3 detach_object hello_box tf_end`
12 `rosrun moveit_python task_generator.py fr3 choose_pipeline OMPL RRTConnect`
13 `rosrun moveit_python task_generator.py fr3 joints_position 0 -1.57 1.57 0 0 0`
14 `rosrun moveit_python task_generator.py fr3 delete_json_sim_content test.json`
15 `rosrun frcobot_hw frcobot_clear_error`
```

3. Use executor json example
```bash
rosrun moveit_python task_executer_json.py fr10 test.json
rosrun moveit_python task_executer_json.py fr10 test_square.json
rosrun moveit_python task_executer_json.py fr10 task_grip_sim.json
```

4. Demo squate test
```bash
rosrun moveit_python task_generator.py fr10 end_coordinate tf_end 0.3 -0.4 0.1 0.0 -0.707 0.0 -0.707
rosrun moveit_python task_executer_json.py fr10 test_square.json
```

### Additional mods

`task_generator.py` provides sevewral gripper orientations.

1. If mode= None:

![alt text](../docs/none.gif)

2. If mode= 'right_angle':

![alt text](../docs/rightangle.gif)

3. If mode= 'vector2point':

![alt text](../docs/vector2point.gif)