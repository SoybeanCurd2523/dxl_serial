#ifndef DXL_SERIAL_H
#define DXL_SERIAL_H

#include "ros/ros.h"
#include "std_msgs/String.h"

// C library headers
#include <stdio.h>
#include <iostream>
#include <string.h>
// #include <Eigen/Dense>
# include <eigen3/Eigen/Dense>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

// typedef unsigned char TYPE;

using namespace std;

class DXL{

protected:
    unsigned char H1 = 0xFF;
    unsigned char H2 = 0xFF;
    unsigned char H3 = 0xFD; 
    unsigned char RSRV = 0x00;

public:
    string port_name;
    int baudrate;

    // sync_write
    
    int motor[100];

    int motor_num;
    
    int motor_id_1;
    int motor_id_2;
    
    int serial_port;
    unsigned int encorder;

    unsigned char CRC_L;
    unsigned char CRC_H;
    unsigned short CRC;
    
    unsigned char buffer_data;
    
    DXL();
    DXL(string port_name_);
    DXL(string port_name_,int baud);
    //~DXL();

    int Initialize();

    unsigned char read_buffer(int serial_port);
    void Torque_On(int motor_id);

    void position(unsigned int encorder);

    void sync_write(VectorXi q);

    // void sync_write(unsigned int encorder_1, unsigned int encorder_2);
    unsigned short update_crc(unsigned char *TxPacket, unsigned short data_blk_size);
};

#endif // DXL_SERIAL_H