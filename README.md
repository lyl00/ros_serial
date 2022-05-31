## How to use this ROS interface for sending and reading from serial port. 

- **Notice**

  Note that this project should base on your own protocol, in this case, we are using the motor driver from Von Tuning(欧拉智能) . But you could write your own code using the same way.

- **Prerequisite**

  Download [serial library](https://github.com/wjwwood/serial), you don't need to build that individually before using that in the ***ros_serial_code***. It is a catkin project so it could be built along side other catkin projects in your workspace. Then download my code to the same src directory. Build them using Catkin make. 

  ``` 
  cd ~/catkin_ws/src
  mkdir my_serial
  git clone ""
  cd ~/catkin_ws
  catkin_make
  ```

  

- **Test**

  1. Connect the serial port to your computer,make sure you are connected by seeing if there  are the **ttyUSB0** or other similar name for your device in the **/dev** folder, the default name for the port is **ttyUSB0**, you could always change this ROS parameter(in source code, rosrun, launch, etc.) 

  2. run the ros node and you could see the feedback in your terminator and you could send command to the motor via the rostopic **/cmd_vel** and read the byte data in **/sensor** topic, note that if you want to print the data, you need to use *"printf"* function''

     ```C++
     cd /dev
     sudo chmod 777 ttyUSB0
     roscore
     rosrun my_serial_node my_serial_node
     
     print example:
     printf( "message sent:%02x", data_to_pub.data.at(0)&0xff);
     printf( "message sent:%02x", data_to_pub.data.at(1)&0xff);
     
     
     //in your subscriber node, you need to translate the std_msg::String to Hex or decimal
     //for example   :
         cout<<"int: "<<hex<<int(data_to_pub.data.at(4)&0xff);
     
     
     
     //command example:
     rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
       x: 1.0
       y: 0.0
       z: 0.0
     angular:
       x: 0.0
       y: 0.0
       z: 0.0" 
           
      //or you can use the keyboard to control the robot by runing:
     rosrun my_serial_node keyboard_teleop_node
     
     ```

     

