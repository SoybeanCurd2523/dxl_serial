#include "dxl_serial.h"

DXL dxl("/dev/ttyUSB0",2000000); 

void moterinfo(){ 
  ROS_INFO("setting dynamixel ID...");
  dxl.motor_id_1 = 1;
  dxl.motor_id_2 = 2;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "dxl_serial_main");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  
  moterinfo();
  ROS_INFO("dxl.motor_id_1 : %d", dxl.motor_id_1);
  ROS_INFO("dxl.motor_id_2 : %d", dxl.motor_id_2);
  
  dxl.serial_port = dxl.Initialize();
  
  ROS_INFO("dxl.serial_port : %d", dxl.serial_port);

  dxl.Torque_On(1);
  dxl.Torque_On(2);

  VectorXi A = VectorXi::Zero(2);
  
  ros::Rate loop_rate(0.5);
  int count = 0;
  while (ros::ok())
  {
    if(count > 4095)
      count = 0;

    // dxl.position(count);
    // dxl.sync_wirte(count, count);
    A(0)=count; 
    A(1)=count;

    dxl.sync_wirte(A);
    
    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count += 500;
  }
}

