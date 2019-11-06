#Image and Laser Scanner editing from rosbag file | ROS

**Author: Lygerakis Fotios | Machine Learning and Robotics Researcher**

**Research Associate at NCRS "Demokritos"**

#### Description
In the `config/config.yaml` there is he `separeted` variable. In the launch file we start the two nodes for which we load the parameter from the configuration file.

* If `separeted == True` the `cameraListener.py` and `scanListener.py` publish an edited version of the subscribed topics `/radio_cam/rgb/image_raw` and `/scan` respectively.
    * Each of them detects motion
    * When motion is detected the cameraListener node publishes on the `/thresholded_image` topic a message, where the moving object is coloured in RGB values and the rest pixels are gray scale.
    * When motion is detected the scanListener node publishes on the `/new_scan` topic a message, where the ranges for the moving object are kept as are and the rest ranges are cast to `MAX_RANGE+1`.
    * if no motion is detected both nodes do not publish anything
* If `separeted == False` a new `uniMSG` is being published, which contains:
        
        Header header 
   
        #LaserScan:
        float32[] distances
        float32[] angles
    
        #Camera:
        uint8[] pixel_values
        
    `distances` and `pixel_values` are being computed as described above and `angles` are being copied from `/scan` topic as is.

#### Running the code
1. In a terminal:
           
        roscore
          
2. In a **new** terminal run the launch file:

        roslaunch human_detect detect.launch            
           
3. In a **new** terminal and play the bag file with -l option to run in loop:

        rosbag play -l ~/path-to-bag_file.bag_file.bag
    
4. In a new terminal open rviz and find the published topics `/thresholded_image` and `new_scan`:

        rviz
  
#### Bonus

The `readBag.py` node reads a bag file with the [ROSBAG Code API](http://wiki.ros.org/rosbag/Code%20API) and performs the editing of the camera Image described above, without the separated functionality. The node reads directly the file and publishes without waiting any clock.
This can be observed by the unstable frame rate on rviz.
#### Running Code
1. In a terminal:
           
        roscore
          
2. In a **new** terminal run the launch file (bag file must be on the same folder):

        roslaunch human_detect bagReader.launch            
    
4. In a new terminal open rviz and find the published topics `/thresholded_image` and `new_scan`:

        rviz
                 