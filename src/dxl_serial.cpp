#include "dxl_serial.h"

int DXL::Initialize(){

  ROS_INFO("Initializing DXL...");
  /*  ROS_INFO : 흰색
      ROS_WARN : 주황색
      ROS_ERROR : 빨간색
  */
  int serial_port = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(serial_port!=-1){
    ROS_INFO("dxl_port_open_success");
  }
  else{
    ROS_ERROR("dxl_port_open_Failed");
  }

  struct termios tty;
   memset( &tty, 0, sizeof(tty) );

  // Check for errors
  if (serial_port < 0) 
      ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
  
  if(tcgetattr(serial_port, &tty) != 0) 
      ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    
  struct termios {
    tcflag_t c_iflag;		/* input mode flags */
    tcflag_t c_oflag;		/* output mode flags */
    tcflag_t c_cflag;		/* control mode flags */
    tcflag_t c_lflag;		/* local mode flags */
    cc_t c_line;			/* line discipline */
    cc_t c_cc[NCCS];		/* control characters */
  };

  switch (this->baudrate)
  {
  case 57600:
    tty.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    break;
  
  case 2000000:
    tty.c_cflag = B2000000 | CS8 | CLOCAL | CREAD;

  default:
    break;
  } 

    tty.c_iflag = IGNPAR;
    tty.c_oflag      = 0;
    tty.c_lflag      = 0;
    tty.c_cc[VTIME]  = 0;
    tty.c_cc[VMIN]   = 0;

   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
        ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  
  return serial_port;
}

// constructor
DXL::DXL(){
    this->port_name = "/dev/ttyUSB0";
    this->baudrate = 20000000;
}
DXL::DXL(string port_name_){
  this->port_name = port_name_;
  this->baudrate = 20000000;
}

DXL::DXL(string port_name_, int baud){
  this->port_name = port_name_;
  this->baudrate = baud;
}

unsigned char DXL::read_buffer(int serial_port){
  
  unsigned char read_buf [256];

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  if (num_bytes < 0)
    ROS_INFO("Error reading: %s", strerror(errno));

  return num_bytes;
}

void DXL::Torque_On(int motor_id){
  unsigned char arr[13] = {H1, H2, H3, RSRV, motor_id, 0x06, 0x00, 
                          0x03, 0x40, 0x00, 0x01};
    ROS_INFO("Torque_On [%d]motor", motor_id);
    // unsigned char arr[13]; 
    // arr[0] = H1;
    // arr[1] = H2;
    // arr[2] = H3;
    // arr[3] = RSRV;
    // arr[4] = motor_id;
    // arr[5] = 0x06; // low byte of length
    // arr[6] = 0x00; // high byte of length
    // arr[7] = 0x03; // instrcuction
    // arr[8] = 0x40; // low byte of starting address
    // arr[9] = 0x00; // high byte of starting address
    // arr[10] = 0x01; // Torque On(size :1byte), 1 : on, 0 :off

  CRC = update_crc(arr, sizeof(arr) - 2);

  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;

  arr[11] = CRC_L;
  arr[12] = CRC_H;
  // for(int i=0; i< sizeof(arr) ; i++)
  // {
  //   ROS_INFO("arr[%d] = 0x%.2X", i, arr[i]);
  // }
  write(serial_port, arr, sizeof(arr));
}

void DXL::position(unsigned int encorder){

  unsigned char position[16] = {H1, H2, H3, RSRV, motor_id_1, 0x09, 0x00, 0x03, 0x74, 0x00};
  ROS_INFO("function position : %d", encorder);

  for(int i=0 ; i<=3 ; i++){
    position[10+i] = ( (encorder >> i * 8) & 0x000000ff );
  }

  CRC = update_crc(position, sizeof(position) - 2);

  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;

  position[14] = CRC_L;
  position[15] = CRC_H;

  // for(int i=0; i< sizeof(position) ; i++)
  // {
  //   ROS_INFO("position[%d] = 0x%.2X", i, position[i]);
  // }

  write(serial_port, position, sizeof(position));
}

void DXL::sync_wirte(VectorXi q){

  int n;
  int size = q.size();
  n = 14 + 5 * size;
  
  unsigned char sync_wirte[n] = {H1, H2, H3, RSRV, 0xFE, 0x11, 0x00, 
                              0x83, 0x74, 0x00, 0x04, 0x00};
  
  //length (packet 5,6)
  for(int i=0 ; i<=1 ; i++){
    sync_wirte[5+i] = ( (n-7 >> i * 8) & 0x000000ff );
  }
  
  //motor id
  for(int i=0; i<size; i++){
      sync_wirte[12 + 5*i] = i+1;
  }

  //encoder
  for(int j=0; j<size; j++){
      for(int i=0 ; i<=3 ; i++){
    sync_wirte[(13 + 5*j) + i] = ( (q[j] >> i * 8) & 0x000000ff );
  }}
  

  CRC = update_crc(sync_wirte, sizeof(sync_wirte) - 2);

  CRC_L = (CRC & 0x00FF);
  CRC_H = (CRC >> 8) & 0x00FF;

  sync_wirte[n-2] = CRC_L;
  sync_wirte[n-1] = CRC_H;

 for(int i=0; i< n ; i++)
  {
    ROS_INFO("position1[%d] = 0x%.2X", i, sync_wirte[i]);
  }
  write(serial_port, sync_wirte, sizeof(sync_wirte));
}

// void DXL::sync_wirte(unsigned int encorder_1, unsigned int encorder_2){
 
//   unsigned char sync_wirte[24] = {H1, H2, H3, RSRV, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00};
//   ROS_INFO("function position1 : %d", encorder_1);
//   ROS_INFO("function position2 : %d", encorder_2);

//   sync_wirte[12] = motor_id_1;
//   sync_wirte[17] = motor_id_2;

//   for(int i=0 ; i<=3 ; i++){
//     sync_wirte[13+i] = ( (encorder_1 >> i * 8) & 0x000000ff );
//   }

//   for(int i=0 ; i<=3 ; i++){
//     sync_wirte[18+i] = ( (encorder_2 >> i * 8) & 0x000000ff );
//   }

//   CRC = update_crc(sync_wirte, sizeof(sync_wirte) - 2);

//   CRC_L = (CRC & 0x00FF);
//   CRC_H = (CRC >> 8) & 0x00FF;

//   sync_wirte[22] = CRC_L;
//   sync_wirte[23] = CRC_H;

// //  for(int i=0; i< sizeof(sync_wirte) ; i++)
// //  {
// //    ROS_INFO("position1[%d] = 0x%.2X", i, sync_wirte[i]);
// //  }

//   write(serial_port, sync_wirte, sizeof(sync_wirte));
// }

unsigned short DXL::update_crc(unsigned char *TxPacket, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_accum = 0;

    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ TxPacket[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
