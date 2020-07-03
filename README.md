# mavros_visp_ompl_exploration
The README.md of the repository (we will see it in our last course lesson), must
contain a comprehensive description on how to download, compile/install and run
your solution. For this reason, in this section you must also add any reference to
external dependencies needed to compile/run the project solution..
# Setup Instructions
It is recommended to use Ubuntu 16.04, gazebo-8 and ROS melodic since all the tests have been made on it. 

# Dependencies Installation
- Download the code in the ROS src folder using 
     ```sh
    $  git clone https://github.com/AlessandroMelone/mavros_visp_ompl_exploration.git
    ```

- Download [jocacace/Firmware][jo_rep] and follow the instruction in the related [README.md][jo_README]. Doing this step, when the istructions 
    ```sh
    $ cd Firmware && make px4_sitl_default
    $ make px4_sitl_default gazebo
    ```
    are meet, you need to generate the sdf file used in this project before to run the previous istructions. 
    
    To generate the desired UAV sdf file you need to replace two files:
    - Replace the file *model_generation.txt* cointained in the folder *Firmware/Tools/sitl_gazebo* with the file of the same name contained in the folder *mavros_visp_ompl_exploration/file_sdf_generation*.
    - Replace the file *tarot.xacro* cointained in the folder *Firmware/Tools/sitl_gazebo/models/rotors_description/urdf/tarot*
    with the file of the same name contained in the folder *mavros_visp_ompl_exploration/file_sdf_generation*. 
    
    Now you can run: 
     ```sh
    $ cd Firmware && make px4_sitl_default
    $ make px4_sitl_default gazebo
    ```

    and continue to read the README of [jocacace/Firmware][jo_rep].
- Install the following packages:
    - OMPL:
    Download the OMPL installation script from this link: https://ompl.
    kavrakilab.org/install-ompl-ubuntu.sh:
        ```sh    
        $ chmod u+x install-ompl-ubuntu.sh
        $ ./install-ompl-ubuntu.sh wil
        $ sudo apt-get install ros-melodic-ompl
        ```
    - visp_auto_tracker:
        ```sh
        sudo apt-get install ros-melodic-visp-auto-tracker
        ```
# Building 
Now you are ready to build the code.
In your ROS src folder execute:
```sh
    $ catkin_make
```
#### Fixes
If the building fails is probably because the messages needed by the code are not created when the code is compiled.
In this case you just need to comment the commands *add_executable(...)*  and *target_link_libraries(...)* in all the *CMakeLists.txt* in the *mavros_visp_ompl_exploration* folder, and repeat the command: 
```sh
    $ catkin_make
```
until the building is 100% complete.

Now the messages files have been created so restore the *CMakeLists.txt* files and build again until the building is 100% complete.
# Running 
- To run the code, first of all launch the Gazebo simulation:
     ```sh    
        $ roslaunch master_pkg tarot.launch 
    ```

- And in another command line windows :
     ```sh    
        $ roslaunch master_pkg all.launch
    ```
Now a menu should appear that allows to start various task.

#### Tips
If you want to have the Px4 package outside of the src folder of your ROS workspace, you could add at your .bashrc the following: 
```sh 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/your_user_name/Firmware/
```
Now to check if all is ok you could execute:
```sh 
$ roscd px4
```
And now you should be in the *Firmware* folder.

[jo_rep]: https://github.com/jocacace/Firmware
[jo_README]: https://github.com/jocacace/Firmware/blob/master/README.md
elf is open source with a [public repository][dill]
 on GitHub.
