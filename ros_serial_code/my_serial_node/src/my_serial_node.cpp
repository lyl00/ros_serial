#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

serial::Serial ros_serial;
#define	sBUFFERSIZE	13//send buffer size
#define	rBUFFERSIZE	256
#define SIZEONEFRAME 36
#define HEAD1  0xAA
#define HEAD2  0x55
#define DRIVER_ADDRESS  0x01
#define CMD_FLAG  0x02// sending linear and angular velocity for the robot.
#define DATA_LENGTH  0x04
#define FUNCTION_ODOM_SET_TO_ZERO  0x07 //brake mode and reset odometry to zero.
#define FUNCTION_RUN_MODE  0x02
#define FUNCTION_BRAKE_MODE  0x06
#define FUNCTION_LOCK_MODE  0x08
#define RESERVE  0x00
#define END  0xCC

using namespace std;
unsigned char s_buffer[sBUFFERSIZE];//发送缓存
char CRC;
int CRC_LEN = 11;
// int l_v=0;
// int r_v=0;
// int l_d=1;
// int r_d=1;
// const float d_between=0.165;//半轮间距
// const float d=0.125;//轮子半径

typedef union{
	unsigned char cvalue[4];
	int ivalue;
}int_union;

// typedef union{
// 	unsigned char cvalue[8];
// 	double ivalue;
// }double_union;

// void data_to_serial(const int left, const int right, const int left_d, const int right_d)
// {
//      int_union left_v,right_v,left_v_d,right_v_d;
//      left_v.ivalue=left;
//      right_v.ivalue=right;
//      memset(s_buffer,0,sizeof(s_buffer));
//      s_buffer[0]=0xAA;
//      s_buffer[1]=0xfe;

//      s_buffer[2]=left_v.cvalue[0];
//      s_buffer[3]=right_v.cvalu
//      left_v_d.ivalue=left_d;e[0];

//      right_v_d.ivalue=right_d;
//      s_buffer[4]=left_v_d.cvalue[0];
//      s_buffer[5]=right_v_d.cvalue[0];

//      s_buffer[6]=0x00;
//      s_buffer[7]=0x00;
//      s_buffer[8]=0x00;
//      s_buffer[9]=0x00;
//      ros_serial.write(s_buffer,sBUFFERSIZE);
// }

//回调函数
unsigned char CRC_cal(unsigned char s_buffer[]){
    int CRC_temp;
    CRC_temp= 0;
    for(int i = 0; i<CRC_LEN;i++)
    {   
        CRC_temp += s_buffer[i];

    }
    std::cout<<"CRC for the current data is :  "<< (CRC_temp&0xFF) << std::endl;
    return CRC_temp&0xFF;
}


void twist_callback(const geometry_msgs::Twist& msg){
    //  cmd_to_serial(msg);
    //  data_to_serial(l_v,r_v,l\\
    // double m/s ->int mm/s: based on the driver protocal.
    int cmd_flag = 2; // sending linear and angular velocity to the robot.
    int_union base_ang_vel;
    base_ang_vel.ivalue =  int(msg.angular.z*1000);
    int_union base_lin_vel ;
    base_lin_vel.ivalue = int(msg.linear.x*1000);
    s_buffer[0] = HEAD1;
    s_buffer[1] = HEAD2;
    s_buffer[2] = DRIVER_ADDRESS;
    s_buffer[3] = CMD_FLAG;
    s_buffer[4] = DATA_LENGTH;
    // H and L 8 bits for linear velocity
    s_buffer[5] = base_lin_vel.cvalue[1];
    s_buffer[6] = base_lin_vel.cvalue[0];
    s_buffer[7] = base_ang_vel.cvalue[1];
    s_buffer[8] = base_ang_vel.cvalue[0];

    s_buffer[9] = FUNCTION_RUN_MODE;
    s_buffer[10] = RESERVE;
    s_buffer[11] = CRC_cal(s_buffer);
    s_buffer[12] = END;
    

    ros_serial.write(s_buffer,sBUFFERSIZE);









 }

string string_to_hex(const string& str){
    string result = "0x";
    char temp;
    stringstream ss;
    for(int i=0;i<str.size();i++)
    {
        ss<<hex<<int(str[i])<<endl;
    
        ss>>temp;
        result+=temp;
    }
    return result;
}


 

int main (int argc, char** argv){
     ros::init(argc, argv, "my_serial_node");
     ros::NodeHandle n;
     std::string port_name;
     int Baudrate;
     n.param<std::string>("port_name", port_name,"/dev/ttyUSB0");
     n.param("Baudrate", Baudrate,115200);
     //subscribe to /cmd_vel to take twist command
     ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1000, twist_callback);
     //publish data motor driver
     ros::Publisher serial_pub = n.advertise<std_msgs::String>("sensor", 1000);
     try
     {
         ros_serial.setPort("/dev/ttyUSB0");
         ros_serial.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_serial.setTimeout(to);
         ros_serial.open();
         
     }
     
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
     if(ros_serial.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }else{
         return -1;
     }


     
     ros::Rate loop_rate(100);
     unsigned char r_buffer[rBUFFERSIZE];
     unsigned char data_one_frame[SIZEONEFRAME];  //36 BYTES
     std_msgs::String data_to_pub;  
     int res,index;
     while(ros::ok()){
         ros::spinOnce();
         if(ros_serial.available()){
             //ROS_INFO_STREAM("Reading from serial port");
             std_msgs::String serial_data;
             string test, data;
             //read data
             
            
            // int len = ros_serial.available();
            // unsigned char r_buffer[len];
            int len = ros_serial.available();
            cout<<"len="<<len<<endl;
            res = ros_serial.read(r_buffer,ros_serial.available());
            // cout<<"1111:"<<sizeof(r_buffer)<<endl;
            for(int i=0;i<len;i++){                
                if(r_buffer[i]==0xaa&&r_buffer[i+1]==0x55){
                    //from last_i to i, construct a string and then send it via
                    index = 0;
                    //serial_pub.publish(data_to_pub);
                    stringstream ss;
                    string tmp;

                    data_to_pub.data.clear();
                    
                    for(int k = 0; k<36;k++){
                        ss<<data_one_frame[k];
                        //printf("%02x  ",data_one_frame[k]);
                        ss>>tmp;
                        ss.clear();
                        //printf("%02x  ", tmp.at(0)&0xff);
                        data_to_pub.data  += tmp;
                    }
                    
                    // cout<<"raw data"<<ss.str()<<endl;
                    // data_to_pub.data = "hello world";
                    serial_pub.publish(data_to_pub);
                    cout<<"size:"<<data_to_pub.data.size();
                    // cout<< "data:"<<hex<< data_to_pub.data;
                    printf( "message sent:%02x", data_to_pub.data.at(0)&0xff);
                    cout<<"int: "<<hex<<int(data_to_pub.data.at(4)&0xff);
                    if((data_to_pub.data.at(0)&0xff)==0xaa)
                        cout<<"true"<<endl;
                    
                    //for(int j = 0;j<SIZEONEFRAME;j++)
                        //printf("%02x  ",data_one_frame[j]);
                   //                 
                    printf("\n ");
                    }
                data_one_frame[index] = r_buffer[i];
                // cout<<"buffer_now"<<r_buffer[i]<<endl;
                // data_to_pub+= r_buffer[i];
                index++;
                // printf("%02x  ",r_buffer[i]);
         }

         //std::cout<<"available data:"<<ros_serial.available()<<endl;
            // int number = ros_serial.available();
            // test = ros_serial.read(number);
            // char stream[number];
            // cout<<test.length();
            
            // int nbytes = hex2bytes(test,stream,number);
            //  data = string_to_hex(test);
            // cout<<nbytes;


            //  ROS_INFO_STREAM("Read: " << std::hex<<serial_data.data);
    //         //  sprintf(&test,"%02x", serial_data.data);
    //              if (nbytes != -1) 
    // {
    //     int i = 0;
    //     for ( ; i < nbytes; ++ i) 
    //     {
    //         printf("%02x ", stream[i]);
    //     }
    

         }
         loop_rate.sleep();
     }
 }

