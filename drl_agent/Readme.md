# Comandos útiles

```
roslaunch drl_agent trainDQN.launch
roslaunch drl_agent trainA2C.launch
roslaunch drl_agent trainQL.launch

roslaunch drl_agent evalDQN.launch
roslaunch drl_agent evalA2C.launch
roslaunch drl_agent evalQL.launch
```

```
killall -9 gzserver gzclient
```

```
rqt_multiplot
```

Hay que cargar la configuración que hay en ```config/rqt_multiplot.xml```

# Rutas útiles

Para modificar velocidad de simulación:
```~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds```
Normales:
```
<real_time_update_rate>1000.0</real_time_update_rate>
<max_step_size>0.001</max_step_size>
```

Aceleradas 20x
```
<real_time_update_rate>0</real_time_update_rate>
<max_step_size>0.003</max_step_size>
```

Para activar/desactivar UI
```
~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch
```
```
<arg name="gui" value="true"/>
```

Por consola se puede reiniciar el mapa (también por tópico):
```
rostopic pub -1 /syscommand std_msgs/String "data: 'reset'"
```

# Links útiles
https://github.com/DLR-RM/stable-baselines3/tree/master/stable_baselines3/common
https://stable-baselines3.readthedocs.io/en/master/index.html
http://joschu.net/docs/nuts-and-bolts.pdf
http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
http://wiki.ros.org/openai_ros
https://gazebosim.org/tutorials?tut=physics_params&cat=physics
