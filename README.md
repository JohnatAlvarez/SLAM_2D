# SLAM 2D con ROS Noetic

Este repositorio contiene instrucciones y código para configurar y ejecutar SLAM (Simultaneous Localization and Mapping) 2D utilizando ROS Noetic en un TurtleBot.

## Instalación de ROS Noetic

Para comenzar, necesitarás instalar ROS Noetic. Aquí están los pasos para la instalación en Ubuntu:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

## Instalación de Paquetes Adicionales

Instala paquetes adicionales necesarios para ROS Noetic:
```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
sudo apt-get install ros-noetic-tf2-sensor-msgs
sudo apt-get install python3-pip 
pip3 install gym==0.21.0 
python3.8 -m pip install GitPython 
python3.8 -m pip install stable_baselines3==1.3.0 
```

## Configuración del Entorno de Trabajo
Configura tu workspace de ROS siguiendo estas instrucciones (Agregar al archivo .bashrc las trutaas donde tenemos paquetes a ejecutar y sourcear la terminal, pues los cambios al modificar el .bashrc se ven en una nueva terminal o luego de sourcear la terminal en uso):
```bash
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/install_isolated/setup.bash 
source ~/Tesis/slam_ia_ws/devel/setup.bash 
source ~/catkin_ws/devel/setup.bash ```

## Ejecución en el TurtleBot Real
```bash
# Iniciar ROS Core desde un PC remoto
    Iniciar una terminal 
    Run roscore from Remote PC.  
    roscore 
    Abrir otra terminal 
    Ssh ubuntu@192.168.203.15 - 192.168.78.15 
    Contraseña: turtlebot 
    catkin_make_isolated --install --use-ninja 
```

# Conectar con TurtleBot a través de SSH (ajusta las direcciones IP según sea necesario)
``bash
Abrir una terminal y verificar ip de master y turtlebot: 
Ifconfig 
En mi caso es 192.168.203.229 
ssh ubuntu@192.168.203.15
Contraseña: turtlebot
roslaunch turtlebot3_bringup turtlebot3_robot.launch

```

##SLAM y Teleoperación
Para iniciar el proceso de SLAM y teleoperar el TurtleBot:
```bash
# Para gmapping:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

# Para cartographer:
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
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer

# Para Karto:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto 

# Para Hector:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector 

# Teleoperación:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

# Comandos útiles:
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch

```

Para el video de funcionamiento referirse a: https://youtu.be/rwX49ddiEcs
