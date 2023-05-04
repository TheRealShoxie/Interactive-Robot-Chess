
# Build and Run

This document consists of a step by step description on how to build and run the program.

Last adapted: 1st of May 2023<br>
Maintainer: Omar Ibrahim(ibo1)

# Build


### ROS

This is based on a Ubuntu 20.04 system. 

Please follow the steps below

1. ***Ros Noeti***<br>

For the installation of ros noetic please follow the installation instructions on
the following website up till point 1.5 excluded: [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation)

2. ***Needed packages***<br>

Please do following command for all the packages listed below:<br> 
> $ sudo apt install <code>package</code> <br>

package list:
- ros-noetic-rviz-visual-tools
- ros-noetic-graph-msgs
- ros-noetic-moveit
- ros-noetic-moveit-visual-tools
- ros-noetic-moveit-resources
- ros-noetic-opencv-apps
- ros-noetic-rgbd-launch
- ros-noetic-franka-description
- ros-noetic-power-msgs
- ros-noetic-robot-controllers

3. ***Initial configuration***<br>

<code> $ rosdep update<br></code>
<code> $ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc<br></code>
<code> $ echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc<br></code>
<code> $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc<br></code>
<code> $ source ~/.bashrc<br></code>

4. ***Catkin workspace***<br>

<code> $ mkdir -p ~/catkin_ws/src<br></code>
<code> $ cd ~/catkin_ws/<br></code>
<code> $ catkin_make<br></code>
<code> $ source devel/setup.bash<br></code>

5. ***ROS kortex***<br>

Please go into your catkin_ws into the src folder

Open a terminal and do following: 

<code> $ sudo apt install python3 python3-pip<br></code>
<code> $ sudo python3 -m pip install conan==1.59<br></code>
<code> $ conan config set general.revisions_enabled=1<br></code>
<code> $ conan profile new default --detect > /dev/null<br></code>
<code> $ conan profile update settings.compiler.libcxx=libstdc++11 default<br></code>
<code> $ git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git<br></code>
<code> $ cd ../<br></code>
<code> $ rosdep install --from-paths src --ignore-src -y<br></code>
<code> $ source devel/setup.bash <br></code>
<code> $ catkin_make <br></code>

5. ***Getting my project***<br>

Please take the ibo1_irc_api folder from DEV/ROS/catkin_ws/src
and place it into your catkin_ws src folder

6. ***Go inside the ibo1_irc_api folder***

7. ***Open CMakeLists.txt***<br>

Please comment out following lines:<br>
 - 139-150
 - 163-168
 - 184-189
 - 248-256

This needs to be done as for C++ libraries the custom msg need to build before the nodes have access to it.

8. ***Build the ibo1_irc_api library***<br>

In the terminal cd back into the catkin_ws folder and run:

<code> $ rosdep install --from-paths src --ignore-src -y<br></code>
<code> $ catkin_make<br></code>

9. ***Remove comments***<br>

Now remove the comments in the CMakeLists.txt from setp 7. <br>

and do step 8. from your catkin_ws folder

10. ***Successfully build***<br>

Now if you have you have no errors the package is correctly build.
Running it is described further below.


### JAVA
To Build my java project navigate to DEV/InteractiveRobotChess.<br>

<em>Prerequisite</em> this was only tested on Java 17.0.6<br>
If you do not have JAVA installed please follow a tutorial for doing so on your environment.


Inside a terminal in this folder run:<br>

Windows:<br>
<code> $ gradlew</code></br>

Linux:<br>
<code> $ ./gradlew</code></br>


# Run

## ROS

To run the rospackage please do following in a linux environment:

Open a terminal and do following:

<code> $ roscore </code>

Open a second tab in the terminal and do:


***Please read the below information before executing this***<br>
<code> $ roslaunch ibo1_irc_api main.launch </code>

You need to supply the filepath to users.txt and chessengines.txt inside the ibo1_irc/data/Users and ibo1_irc_api/data/Chess folder to the program.<br>
This can be done in two ways:<br>
 - Add as ros launch param when launching
 - Or By editing the main.launch file and changing the fault value for chessEngineNamesFilePath and usersNameFilePath inside the ibo1_irc_api/launch folder.

 > Please supply the filepaths from root example "/.../.../...txt"

For information on how to add arguments to a roslaunch please refer to: [Roslaunch Explanation](http://wiki.ros.org/roslaunch/Commandline%20Tools). Refer specifically to 1.2 Passing in args.

Possible arguments to set when launching the file:
 - start_rviz [true|false] - Used for starting rviz or not
 - systemDebug [true|false] - Used for enabling/disabling system Debugging
 - chessEngineNameFilePath [FilePath] - Used for settting the chessEngines.txt file path 
 - usersNameFilePath [FilePath] - Used for setting the users.txt file path
 - imageProcessingMaxCorners [int] - Used for telling the image Processing how many corners to look for. For expected usage keep at a 100


### Running the ROS Tests
Open a terminal inside your catkin_ws folder. Execute following command:
<code> $ catkin_make run_tests_ibo1_irc_api </code>

Ensure that you are running roscore when executing this command.


## JAVA

Navigate to the InteractiveRobotChess folder.<br> 
 
Inside of that run following command through a terminal:<br>

Windows:<br>
<code> $ gradlew run</code></br>

Linux:<br>
<code> $ ./gradlew run</code></br>

### Logging in
For logging in please use Username: Tester and password: tester. There is no difference between admin and user in the current iteration.

### Other
Please ensure that the ROS system is running and that the socket couldn't be opened. Further please ensure that the gazebo environment started before logging in.


# Problems

If problems arise please get in contact with Omar ibrahim [ibo1@aber.ac.uk](ibo1@aber.ac.uk).