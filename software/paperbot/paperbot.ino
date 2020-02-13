/*
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

//////////////
// Includes //
//////////////    // TODO: Separate functions into proper header and cpp files

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>
#include <Wire.h>

#include "debug.h"
#include "file.h"
#include "server.h"
#include "VL53L0X.h"

///////////////////////////////
//  MPU I2C Interface Setup  //
///////////////////////////////

// Define MPU I2C and Register Addresses
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12

// Sensor Calibration Values m-mag g-gyro
int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;

const short int mx_offset = -40.75;
const short int my_offset = +99.5;

// I2C Functions
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

///////////////////
// VL53L0X Setup //
///////////////////
// Linear Utilizes VL53L0X Library Functions
VL53L0X sensor;
VL53L0X sensor2;

////////////////////////////
//  Wifi Interface Setup  //
////////////////////////////

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 88;  //initially 90, calibrated to eliminate startup hum
int servo_right_ctr = 88;

// Data Transmission parameters
unsigned long TIME_INTERVAL = 200;  // time between sensor data captures in millis
unsigned long run_time = 0;
unsigned long current_time;
static bool startData = false;


// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;

///////////////////////
//  Arduino Setup()  //
///////////////////////

void setup() {
    setupPins();

    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    // Initialize MPU operation
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

    // Initialize gyro
    // Set by pass mode for the gyro sensors
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
    // Set resolution of gyro output
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);

    // Initialize Linear Sensors    
    digitalWrite(D3, HIGH);
    delay(150);
    
    sensor.init(true);
    delay(100);
    sensor.setAddress((uint8_t)22);
  
    digitalWrite(D4, HIGH);
    delay(150);
    sensor2.init(true);
    delay(100);
    sensor2.setAddress((uint8_t)25);

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }


    
    // Initizalize wifi communication
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);


    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();
}

//////////////////////
//  Arduino Loop()  //
//////////////////////

void loop() {
    wsLoop();
    httpLoop();

    // check if half a second has passed, if so, collect and transmit data
    // Time between captures can be changed in Wifi Setup at TIME_INTERVAL
    current_time = millis();
    if(startData && current_time > (run_time + TIME_INTERVAL)) {
        // Sensor Data
        int16_t magx;
        int16_t magy;
        int16_t magz;
        float   theta;

        int16_t grx;
        int16_t gry;
        int16_t grz;
        
        uint16_t front;
        uint16_t right;
        

        // Collect Data
        getMagnetometer(&magx, &magy, &magz, &theta);
        getGyro(&grx, &gry, &grz);
        getLinear(&front, &right);
        
        // Transmit Data
        String temp = String(magx) + "\t" + String(magy) + "\t" + String(magz) + "\t" +  String(theta) + "\t|\t" + String(grx) + "\t" + String(gry) + "\t" + String(grz) + "\t|\t" + String(front) + "\t" + String(right);
        wSend(temp);
        
        run_time = current_time;
    }
}


////////////////////////
// Movement Functions //
////////////////////////

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  DEBUG("left");
  drive(180, 180);
}

void right() {
  DEBUG("right");
  drive(0, 0);
}

///////////////////////
// Sensing Functions //
///////////////////////
void wSend(String txt)
{
    char buf[80];    //TODO (optional) change to constant BUFFERSIZE
    txt.toCharArray(buf, 80);
    wsSend(0, buf);
}

void getMagnetometer(int16_t *mgx, int16_t *mgy, int16_t *mgz, float *th) {
    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    
    // Read register Status 1 and wait for the DRDY: Data Ready
    
    uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));
  
    // Read magnetometer data  
    uint8_t Mag[7];  
    I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
    // Create 16 bits values from 8 bits data
    
    // Magnetometer
    int16_t mx=(Mag[1]<<8 | Mag[0]);
    int16_t my=(Mag[3]<<8 | Mag[2]);
    int16_t mz=(Mag[5]<<8 | Mag[4]);
  
    float heading = atan2(mx + mx_offset, my + my_offset);
  
    // Once you have your heading, you must then add your 'Declination Angle',
    // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
    // Find yours here: http://www.magnetic-declination.com/
    
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.207403293;
    heading += declinationAngle;
  
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
  
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
  
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI; 
  
    *mgx = mx;
    *mgy = my;
    *mgz = mz;
    *th  = headingDegrees;
    return;
}

void getGyro(int16_t *gyx, int16_t *gyy, int16_t *gyz) {
    // Read gyro data  
    uint8_t Gyro[7];
    I2Cread(MPU9250_ADDRESS,0x43,7,Gyro);
  
    // Create 16 bits values from 8 bits data
    
    // Gyroscope
    int16_t gx=(Gyro[0]<<8 | Gyro[1]);
    int16_t gy=(Gyro[2]<<8 | Gyro[3]);
    int16_t gz=(Gyro[4]<<8 | Gyro[5]);

    *gyx = gx;
    *gyy = gy;
    *gyz = gz;
    return;
}

void getLinear(uint16_t *fr, uint16_t *ri) {
    // Retrieve linear sensor output, does not check for timeouts
    *fr = sensor.readRangeSingleMillimeters();
    *ri = sensor2.readRangeSingleMillimeters();

    return;
}

///////////
// Setup //
///////////

void setupPins() {
    // setup Serial, I2C Interface, LEDs and Motors
    Wire.begin(SDA_PORT,SCL_PORT);
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    // Linear Sensor Pins
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    digitalWrite(D7, LOW);
    digitalWrite(D8, LOW);

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {  // Wifi transaction handler
                                                                                    // Called once every web event
    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '#') {
                // Set Heart Button to Start/Stop Sensor Data Transmission
                if(payload[1] == 'C') {
                  LED_ON;
                  if(!startData) {
                     wsSend(id, "Starting Data Tranmission");
                     wsSend(id, "Format:");
                     wsSend(id, "mx\tmy\tmz\tdeg\t|\tgx\tgy\tgz\t|\tf\tr");
                     startData = !startData;
                  }
                  else {
                     wsSend(id, "Ending Data Transmission");
                     wsSend(id, "mx\tmy\tmz\tdeg\t|\tgx\tgy\tgz\t|\tf\tr");
                     startData = !startData;
                  }
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}
