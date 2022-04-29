#include "dxl_serial.h"

DXL dxl("/dev/ttyUSB1",1000000); 

void moterinfo(){ 
  dxl.motor_id = 1;
  dxl.motor_id_2 = 2;
}

int main(int argc, char **argv)
{ 
  // unsigned int encorder = 2048
  // ROS_INFO("encorder : %d", encorder);
  // ROS_INFO("encorder : %x", encorder);
  // ROS_INFO("encorder : %08x", encorder);

  // for(int cnt=3 ; cnt>=0 ; cnt--){
  //   ROS_INFO("%02x", (encorder >> cnt * 8) & 0x000000ff);
  // }
  ros::init(argc, argv, "dxl_serial_main");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  
  moterinfo();
  ROS_INFO("dxl.motor_id : %d", dxl.motor_id);
  // ROS_INFO("dxl.baudrate : %d", dxl.baudrate);
  
  dxl.serial_port = dxl.Initialize();
  
  ROS_INFO("dxl.serial_port : %d", dxl.serial_port);

  dxl.Torque_On();

  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    if(count > 4095)
      count = 0;

    dxl.position1(count);
    // dxl.sync_wirte(count, count);
    
    dxl.buffer_data = dxl.read_buffer(dxl.serial_port);

    // for(unsigned char i=0 ; i< sizeof(dxl.buffer_data) ; i++)
    //   {
    //     ROS_INFO("[0x%c] ",dxl.buffer_data[i]);
    //   }

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count += 1000;
  }

}

