# SLAM_2D
IMplementación y puesta a punto de los algoritmos propuesto en el documento de Tésis

sudo apt-get install ros-noetic-tf2-sensor-msgs 

 

Al realizar catkin:MAKE en la IA 
 

sudo apt-get install python3-pip 

pip3 install gym==0.21.0 

python3.8 -m pip install GitPython 

python3.8 -m pip install stable_baselines3==1.3.0 

 

Configurar mi ws home/jo-alvarez/Tesis/slam_ia_ws/ en el archivo de gmapping_turtlebot3.yaml 
 
Para  cartographer 
cd /home/jo-alvarez/catkin_ws 
catkin_make_isolated --install --use-ninja 
 

 

Al montarlo en el turtlebot real,  

Iniciar una terminal 

Run roscore from Remote PC.  

$ roscore 

Abrir otra terminal 

Ssh ubuntu@192.168.203.15 - 192.168.78.15 

Contraseña: turtlebot 

catkin_make_isolated --install --use-ninja 

 

 

PAra gmapping: 
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping 
PAra cartographer 

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer 
 

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto 
 

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector 
 

Teleop: 

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
 
Al iniciar: 
source /opt/ros/noetic/setup.bash 

source ~/catkin_ws/install_isolated/setup.bash 

source ~/Tesis/slam_ia_ws/devel/setup.bash 

source ~/catkin_ws/devel/setup.bash 

 
 
PAra Cartographer: 
ERROR: the following packages/stacks could not have their rosdep keys resolved 

to system dependencies: 

cartographer: [libabsl-dev] defined as "not available" for OS version [focal] 

Se modifica el archivo package.xml en la linea 46: 

En donde se elimina lo siguiente:   <depend>libabsl-dev</depend>  

Al eliminar esa línea, encuentro que: 
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 

#All required rosdeps installed successfully 

Catkin_make_isolated 

 

source ~/catkin_ws/devel_isolated/setup.bash 
 
TUtorial para ejecutarlo desde 0: 

Abrir una terminal y verificar ip de master y turtlebot: 

Ifconfig 

EN mi caso es 192.168.154.229 
 

Luego lo modifico, en dado caso que lo requiera, para cambiar la ip así: 

nano .bashrc 

Y verifico que esté así(o sea con la ip que encontramos en el paso anterior): 

 

Al realizar los cambios procedo a precionar Crtl + o, luego Enter y después Crtl + x 

Luego de esto, procedemos a ejecutar roscore, se debe ver algo así: 
 

Posterior a eso, es necesario encender el robot, y conectarnos via ssh al turtlebot y lo hacemos sabiendo la dirección del turtlebot, además de que deben estar conectados a la misma red, abrimos una terminak y ejecutamos el siguiente comando: 
ssh ubuntu@192.168.154.15  

Y cuando nos solicite la contraseña es: “turtlebot” y debe aparecer así para confirmar que está bien, también se realiza el proceso de if config para configurar las direcciones de ros del maestro pc y turtlebot. 

 

Y se ejecuta allí la instrucción de roslaunch turtlebot3_bringup turtlebot3_robot.launch y debe verse al final así, para verificar que lo realiza correctamente: 

 

 

Ahora abro una nueva terminal, que será en la que me conectaré al robot para moverlo: 

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

 

A continuación ejecutaremos la tarea de slam con siguiente comando: 

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping 

 

En slam_methods:= [frontier_exploration, hector, gmapping, karto, cartographer] 

 
 

AHora nos haremos sobre la terminal de teleoperación y manejaremos el robot, hasta completar el mapa, visto en rviz, posterior a ello, guardaremos el mapa. 

 

rosrun map_server map_saver -f my_map 

 
