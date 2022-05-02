#include "dxl_serial.h"

DXL dxl("/dev/ttyUSB0",2000000); 

void moterinfo(){ 
  ROS_INFO("setting dynamixel ID...");

  dxl.motor_num = 3;
  for(int i=0 ; i< dxl.motor_num ; i++){
    dxl.motor[i] = i+1;
  }
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "dxl_serial_main");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  moterinfo();

  for(int i=0 ; i< dxl.motor_num ; i++){
    ROS_INFO("dxl.motor[%d] : %d", i, i+1);
  }

  dxl.serial_port = dxl.Initialize();
  
  ROS_INFO("dxl.serial_port : %d", dxl.serial_port);

  for(int i=1 ; i<= dxl.motor_num ; i++){
      dxl.Torque_On(i);
  }


  VectorXi q = VectorXi::Zero(dxl.motor_num);
  
  ros::Rate loop_rate(50);
  int count = 0;
  
  while (ros::ok())
  {
    if(count > 4095)
      count = 0;

    // dxl.position(count);
    // dxl.sync_write(count, count);

        dxl.encorder2 = round(2047*(-sin(2 * M_PI * count / 1000 + M_PI / 2)) + 2047); //hs
      for(int i=0 ; i < dxl.motor_num ; i++){
        q(i) = dxl.encorder2;
      }   

      dxl.sync_write(q);


    // dxl.sync_write(A);
    
    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    //count += 500;
    
    
    count++;
  }
}

