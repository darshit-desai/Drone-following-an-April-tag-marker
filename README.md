# Drone following an April tag marker
 A script written in C++ which allows a drone to follow the april tag autonomously in offboard mode
 
 ## Flight test video
 
 The following video shows the drone being tested with an April tag marker. The april tag marker is slowly pulled and the drone using it's downward facing tracking camera captures the April tag and calculates it's pose in the body frame of the camera. This pose is subsequently rotated and converted from camera to the drone frame and from drone frame to the local inertial frame for the tracking of April tag.
 
 
 
 ## Code run instructions (Note for the following steps to work a linux docker with ros-melodic and opencv1.2 installed on it should be present on board the drone's computer)
 
   * To run the ros node, Download the package from the below command:
 
          git clone https://github.com/darshit-desai/Drone-following-an-April-tag-marker
      
  * After this copy and paste the package in your catkin workspace inside the ros-melodic docker on the onboard computer of the voxl m500 drone  

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash
        roslaunch apriltag_coord joy_apriltag_xcoord_mavros.launch
    * In another terminal with voxl docker logged in:    
          
          rosrun offboard offboard_node

  * Take off the drone in position control mode after reaching the desired height and length from the April tag flip it to offboard mode and let the script run and publish drone pose data as the tag is moved. Press Ctrl+C on both docker terminals once you are done with the tracking.
  * Flip the mode to position mode and land the drone
