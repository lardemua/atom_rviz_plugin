#Rviz Panel Plugin - ATOM Calibration
An interface to help with all the stages of the [ATOM calibration](https://github.com/lardemua/atom) project.

## Installations
Before using this package, it is necessary to do the installation of the atom package, which can be found [here](https://github.com/lardemua/atom).

There you will find the instructions to follow for your robotic system. The robotic system used during this project was the mmtbot, which you can find [here](https://github.com/miguelriemoliveira/mmtbot).

## Build
Having done the installations mentioned above, just go to your workspace and clone this repository and then build the package, like this:

    cd <catkin_ws>/src
    git clone https://github.com/MiguelDRPina/atom_rviz_plugin
    cd ~/catkin_ws && catkin_make

## Usage
### - First Step
#### The first step was creating a panel on rviz with a simple button that prints something in the terminal

![challenge_1](https://user-images.githubusercontent.com/73201389/109989382-5ab1b600-7d00-11eb-93f9-f79f63dd2ed2.gif)

### - Second Step
#### Creating a tab on the panel dedicated to reading and writing all the parameters of the configuration file (config.yml)
![challenge_2](https://user-images.githubusercontent.com/73201389/115042207-2cacbd80-9ecb-11eb-8a40-75eb317649d8.gif)
### - Third Step
#### Creating a tab on the panel for setting the initial estimate of the sensors
![challenge_3](https://user-images.githubusercontent.com/73201389/115042752-b0ff4080-9ecb-11eb-854c-a0d140c2afcd.gif)
### - Fourth Step
#### Creating a tab on the panel for collecting the data of all sensors
