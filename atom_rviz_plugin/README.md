### First Step
#### Creating a panel on rviz with a simple button that prints something in the terminal

![challenge_1](https://user-images.githubusercontent.com/73201389/109989382-5ab1b600-7d00-11eb-93f9-f79f63dd2ed2.gif)

#### Build
    cd <catkin_ws>/src
    git clone https://github.com/MiguelDRPina/RVIZ_Plugin_SensorCalibration
    cd ~/catkin_ws && catkin_make

#### Run
First terminal:
    
    roscore

Second terminal:

    rosrun rviz rviz