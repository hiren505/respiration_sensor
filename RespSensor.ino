/* radar constants */
#define BUF_SIZE 32

// Flag Byte codes
const unsigned  char  startByte   = 0X7D;
const unsigned  char  stopByte    = 0X7E;
const unsigned  char  escapeByte  = 0X7F;

// Byte code to reset the module
const unsigned  char  XTS_SPC_MOD_RESET = 0X22;

// Byte code for acknowledgement
const unsigned  char  XTS_SPR_ACK       = 0X10;

// Byte codes to inform the host regarding bootup status
const unsigned  char  XTS_SPR_SYSTEM    = 0X30;
const unsigned  char  XTS_SPRS_BOOTING  = 0X10;
const unsigned  char  XTS_SPRS_READY    = 0X11;

// Ping commands to check connection to the module
const unsigned  char  XTS_SPC_PING              = 0X01;        // ping command code
const unsigned  long  XTS_DEF_PINGVAL           = 0Xeeaaeaae;  // ping seed value
const unsigned  char  XTS_SPR_PONG              = 0X01;        // pong response code
const unsigned  long  XTS_DEF_PONGVAL_READY     = 0xaaeeaeea;  // module is ready
const unsigned  long  XTS_DEF_PONGVAL_NOTREADY  = 0xaeeaeeaa;  // module is not ready

// Byte codes to load the application
const unsigned  char  XTS_SPC_MOD_LOADAPP = 0X21;

// Byte codes to execute the application
const unsigned  char  XTS_SM_RUN  = 0X01; // Sensor module in running mode
const unsigned  char  XTS_SM_IDLE = 0X11; // Idle mode. Sensor module ready but not active
const unsigned  char  XTS_SPC_MOD_SETMODE = 0X20;

// Byte codes for LED control
const unsigned  char  XTS_SPC_MOD_SETLEDCONTROL = 0X24;
const unsigned  char  XT_UI_LED_MODE_OFF        = 0X00;
const unsigned  char  XT_UI_LED_MODE_SIMPLE     = 0X01;
const unsigned  char  XT_UI_LED_MODE_FULL       = 0X02;

// Byte codes for Error Handling
const unsigned  char  XTS_SPR_ERROR = 0X20;
const unsigned  char  XTS_SPRE_NOT_RECOGNIZED = 0X01; // command not recognized
const unsigned  char  XTS_SPRE_CRC_FAILED     = 0X02; // checksum failed
const unsigned  char  XTS_SPRE_APP_INVALID    = 0X20; // command recognized, but invalid

// Application commands
const unsigned  char  XTS_SPC_APPCOMMAND    = 0X10;
const unsigned  char  XTS_SPCA_SET          = 0X10;

// Byte code to set Detection zone
const unsigned  long  XTS_ID_DETECTION_ZONE  = 0X96a10a1c;

// Byte code to set Sensitivity
const unsigned  long  XTS_ID_SENSITIVITY  = 0x10a5112b;

// Byte codes for Respiration application
const unsigned  long  XTS_ID_APP_RESP     = 0X1423a2d6;
const unsigned  char  XTS_SPR_APPDATA     = 0X50;
const unsigned  long  XTS_ID_RESP_STATUS  = 0X2375fe26;

//Detection Zone constants
const long DETECTIONZONE_0_5m = 0X3f000000;
const long DETECTIONZONE_1_2m = 0X3f99999a;
const long DETECTIONZONE_1_0m = 0X3f800000;
const long DETECTIONZONE_1_7m = 0X3fd9999a;
const long DETECTIONZONE_1_8m = 0X3fe66666;
const long DETECTIONZONE_2_5m = 0X40200000;

//Sensitivity constants
const long SENSITIVITY  = 0X0000005;

unsigned char recv_buf[BUF_SIZE];   // Buffer for receiving data from radar. Size is 32 Bytes
  

//function prototypes
void send_command(const unsigned char * cmd, int len);
void flush_buffer();
void reset_module();
int receive_data();
void pingModule();
void load_respiration_app();
void execute_app();
void get_resp_state();
void set_detetction_zone();
void set_sensitivity();
void get_resp_movement();
void empty_serial_buffer();
float hexToFloat(unsigned char* hexPtr);


//function definitions
float hexToFloat(unsigned char* hexPtr){
 
 unsigned char hexToInt[4] = {0};
 unsigned int  signbit;
 unsigned int  exponent;
 unsigned int  mantissa;
 unsigned int  uintVal;
 unsigned int* uintPtr = NULL;
 
 for(int x = 0; x < 4; x++){
  hexToInt[x] = *(hexPtr);
  hexPtr++;
 }

 uintPtr = (unsigned int*)(hexToInt);
 uintVal  = *uintPtr;
 signbit =  (uintVal & 0x80000000) >> 31;
 exponent = (uintVal & 0x7F800000) >> 23 ;
 mantissa = (uintVal & 0x007FFFFF) | 0x00800000;
  
 return (float)((signbit == 1)?-1.0:1.0)*mantissa / pow(2.0, (127 - exponent + 23));
}


void empty_serial_buffer() {

    while(Serial1.available())
    {
      Serial1.read();
    }
    Serial.print("\n Serial buffer is emptied...\n");
}


void flush_buffer() {

  for (int x = 0; x < BUF_SIZE; x++) {
    recv_buf[x] = 0;
  }
  Serial.print("\n receive buffer flushed...\n");
  return;
}


void send_command(const unsigned char * cmd, int len) {

  // Calculate CRC
  char crc = startByte;
  for (int i = 0; i < len; i++) {
    crc ^= cmd[i];
  }

  // Send startByte + Data + crc + stopByte
  Serial1.write(startByte);
  Serial1.write(cmd, len);
  Serial1.write(crc);
  Serial1.write(stopByte);

}


void reset_module()
{
  send_command(&XTS_SPC_MOD_RESET, 1);

  delay(1000);
  receive_data(); // acknowledgement
  delay(1000);
  receive_data(); // booting state
  delay(1000);
  receive_data(); // ready state
}


int receive_data() {

  // Get response
  char last_char = 0x00;
  int recv_len = 0; //Number of bytes received

  //while(!Serial1.available());

  //Wait for start character
  while (1)
  {
    char c = Serial1.read();  // Get one byte from radar

    if (c == escapeByte)
    {
      // If it's an escape character –
      // ...ignore next character in buffer
      //c = Serial1.read();
      continue;
    }
    else if (c == startByte)
    {
      // If it's the start character –
      // ...we fill the first character of the buffer and move on
      recv_buf[0] = startByte;
      recv_len = 1;
      break;
    }
  }

  // Start receiving the rest of the bytes
  while (1)
  {
    // read a byte
    char cur_char = Serial1.read();  // Get one byte from radar

    if (cur_char == -1)
    {
      continue;
    }

    // Fill response buffer, and increase counter
    recv_buf[recv_len] = cur_char;
    recv_len++;

    // is it the stop byte?
    if (cur_char == stopByte)
    {
      if (last_char != escapeByte)
        break;  //Exit this loop
    }

    // Update last_char
    last_char = cur_char;
  }

  // Calculate CRC
  char crc = 0;
  char escape_found = 0;

  // CRC is calculated without the crc itself and the stop byte, hence the -2 in the counter
  for (int i = 0; i < recv_len - 2; i++)
  {
    // We need to ignore escape bytes when calculating crc
    if (recv_buf[i] == escapeByte && !escape_found)
    {
      escape_found = 1;
      continue;
    }
    else
    {
      crc ^= recv_buf[i];
      escape_found = 0;
    }
  }


  // Check if calculated CRC matches the recieved
  if (crc == recv_buf[recv_len - 2])
  {
    Serial.print("\n Received Data is Correct\n");

    for (int i = 0; i < BUF_SIZE; i++)
    {
      Serial.printf("%x  ", recv_buf[i]);
    }

    Serial.print("\n Received All Data \n");
    return 0;  // Return 0 upon success
  }
  else
  {
    Serial.print("\n Received Data is Not Correct \n");

    //delay(1000);
    return -1; // Return -1 upon crc failure
  }

}



/* currently not working as command is not recognized
  void pingModule() {
  char crc = startByte;
  crc ^= XTS_SPC_PING;
  crc ^= XTS_DEF_PINGVAL;

  // Send xt_start + command + crc_string + xt_stop
  Serial1.write(startByte);
  Serial1.write(XTS_SPC_PING);
  Serial1.write(XTS_DEF_PINGVAL);
  Serial1.write(crc);
  Serial1.write(stopByte);
 delay(1000);
  receive_data();

  }*/



void load_respiration_app() {
  //Fill send buffer
  unsigned char send_buf[5];
  send_buf[0] = XTS_SPC_MOD_LOADAPP;
  send_buf[4] = (XTS_ID_APP_RESP >> 24) & 0xff;
  send_buf[3] = (XTS_ID_APP_RESP >> 16) & 0xff;
  send_buf[2] = (XTS_ID_APP_RESP >> 8) & 0xff;
  send_buf[1] =  XTS_ID_APP_RESP & 0xff;

  //Send the command
  send_command(send_buf, 5);

  delay(1000);
  receive_data();
}



void set_detetction_zone() {
  //Fill send buffer
  unsigned char send_buf[14];

  send_buf[0] = XTS_SPC_APPCOMMAND;
  send_buf[1] = XTS_SPCA_SET;

  send_buf[5] = (XTS_ID_DETECTION_ZONE >> 24) & 0xff;
  send_buf[4] = (XTS_ID_DETECTION_ZONE >> 16) & 0xff;
  send_buf[3] = (XTS_ID_DETECTION_ZONE >> 8) & 0xff;
  send_buf[2] =  XTS_ID_DETECTION_ZONE & 0xff;

  send_buf[9] = (DETECTIONZONE_0_5m >> 24) & 0xff;
  send_buf[8] = (DETECTIONZONE_0_5m >> 16) & 0xff;
  send_buf[7] = (DETECTIONZONE_0_5m >> 8) & 0xff;
  send_buf[6] =  DETECTIONZONE_0_5m & 0xff;

  send_buf[13] = (DETECTIONZONE_1_2m >> 24) & 0xff;
  send_buf[12] = (DETECTIONZONE_1_2m >> 16) & 0xff;
  send_buf[11] = (DETECTIONZONE_1_2m >> 8) & 0xff;
  send_buf[10] =  DETECTIONZONE_1_2m & 0xff;

  //Send the command
  send_command(send_buf, 14);

  delay(1000);
  receive_data();

}


void set_sensitivity() {
  //Fill send buffer
  unsigned char send_buf[10];

  send_buf[0] = XTS_SPC_APPCOMMAND;
  send_buf[1] = XTS_SPCA_SET;

  send_buf[5] = (XTS_ID_SENSITIVITY >> 24) & 0xff;
  send_buf[4] = (XTS_ID_SENSITIVITY >> 16) & 0xff;
  send_buf[3] = (XTS_ID_SENSITIVITY >> 8) & 0xff;
  send_buf[2] =  XTS_ID_SENSITIVITY & 0xff;

  send_buf[9] = (SENSITIVITY >> 24) & 0xff;
  send_buf[8] = (SENSITIVITY >> 16) & 0xff;
  send_buf[7] = (SENSITIVITY >> 8) & 0xff;
  send_buf[6] =  SENSITIVITY & 0xff;

  //Send the command
  send_command(send_buf, 10);

  delay(1000);
  receive_data();

}


// Execute respiration application
void execute_app() {
  //Fill send buffer
  unsigned char send_buf[2];
  send_buf[0] = XTS_SPC_MOD_SETMODE;
  send_buf[1] = XTS_SM_RUN;

  //Send the command
  send_command(send_buf, 2);

  delay(1000);
  receive_data();
  
}


void get_resp_movement() {

  receive_data();
  float * movement;

  //Point float pointer to the first byte of the float in the buffer
  movement = (float*)& recv_buf[22];
  Serial.printf("\n movement = %.2f mm \n", *movement);

}


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(115200);

  empty_serial_buffer();
  
  // start reset
  Serial.print("\n Starting RESET......\n");
  reset_module();
  Serial.print("\n RESET Successfull...\n");
  flush_buffer();

  // load application
  Serial.print("\n Loading Respiration APPLICATION......\n");
  load_respiration_app();
  Serial.print("\n Loading APPLICATION Successfull...\n");
  flush_buffer();

  // set detection zone
  Serial.print("\n Setting Detection Zone......\n");
  set_detetction_zone();
  Serial.print("\n Detection Zone Set Sucessfully...\n");
  flush_buffer();

  // sensitivity
  Serial.print("\n Setting Sensitivity......\n");
  set_detetction_zone();
  Serial.print("\n Sensitivity Set Sucessfully...\n");
  flush_buffer();

  // execute application
  Serial.print("\n Executing Respiration APPLICATION......\n");
  execute_app();
  Serial.print("\n Executing APPLICATION Successfull...\n");

  empty_serial_buffer();
  flush_buffer();
}



void loop() {
    // put your main code here, to run repeatedly:
    float distance = 0.0; 
    float movement = 0.0;
    int state_data = 0;
    int rpm = 0;
    
    empty_serial_buffer();
    delay(500);
    receive_data();
    state_data = (int)recv_buf[10];
    Serial.printf("\n state data = %d \n",state_data);
    if(state_data == 0)
    {
      rpm = (int)recv_buf[14];
      distance = hexToFloat(&recv_buf[18]);
      movement = hexToFloat(&recv_buf[22]);
      Serial.printf("\n rpm = %d\t\tdistance = %.2fm\t\tmovement = %.2fmm\n",rpm,distance,movement);
    }
    //flush_buffer();
 }
 
