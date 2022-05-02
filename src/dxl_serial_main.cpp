#include "dxl_serial.h"

DXL dxl("/dev/ttyUSB0",2000000); 

void moterinfo(){ 
  ROS_INFO("setting dynamixel ID...");

  dxl.motor_num = 3;
  for(int i=1 ; i<= dxl.motor_num ; i++){
    dxl.motor[i] = i;
  }
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

  for(int i=1 ; i<= dxl.motor_num ; i++){
      dxl.Torque_On(i);
  }


  VectorXi q = VectorXi::Zero(dxl.motor_num);
  
  ros::Rate loop_rate(0.5);
  int count = 0;
  while (ros::ok())
  {
    if(count > 4095)
      count = 0;

    // dxl.position(count);
    // dxl.sync_write(count, count);

      for(int i=0 ; i< dxl.motor_num ; i++){
        q(i) = count;
      }   

      dxl.sync_write(q);


    // dxl.sync_write(A);
    
    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count += 500;
  }
}

