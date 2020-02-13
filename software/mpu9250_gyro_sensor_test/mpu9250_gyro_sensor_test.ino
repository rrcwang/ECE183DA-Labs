/*MPU9250 Sensor Test Code
 * By: stevenvo
 * Original Source: https://github.com/stevenvo/mpuarduino/blob/master/mpuarduino.ino
 * Edited for EE183DA by Nathan Pilbrough
 * 
 * Description: Basic code to test the functionality of the
 * MPU9250 sensor. Refer to the startup guide on CCLE for 
 * more information. NOTE: this is meant to help confirm 
 * communication with the sensor, calibration is still required
 */
#include <Wire.h>
#include <Servo.h>


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

int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;

// Initialize servo variables
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;


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

//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  drive(servo_left_ctr, servo_right_ctr);
}

void forward() {
  drive(0, 180);
}

void backward() {
  drive(180, 0);
}

void left() {
  drive(180, 180);
}

void right() {
  drive(0, 0);
}

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Set by pass mode for the gyro sensors
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

  // Set resolution of gyro output
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  
  // Request first gyro single measurement
  // I2CwriteByte(GYRO_FULL_SCALE_1000_DPS,0x0A,0x01);

  // Set up servo pins
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);

  delay(3000);
}

long int cpt=0;
// Main loop, read and display data
void loop()
{
  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");

 
  // _____________________
  // :::  Gyroscope ::: 

  // Request first gyro single measurement
  //I2CwriteByte(GYRO_FULL_SCALE_250_DPS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  /*uint8_t ST1;
  do
  {
    I2Cread(GYRO_FULL_SCALE_250_DPS,0x02,1,&ST1);
  }
  while (!(ST1&0x01)); */

  // Read gyro data  
  uint8_t Gyro[7];
  I2Cread(MPU9250_ADDRESS,0x43,7,Gyro);

  // Create 16 bits values from 8 bits data
  
  // Gyroscope
  int16_t gx=(Gyro[0]<<8 | Gyro[1]);
  int16_t gy=(Gyro[2]<<8 | Gyro[3]);
  int16_t gz=(Gyro[4]<<8 | Gyro[5]);

  // _______________
  // ::: Calibration Phase :::

  if(cpt < 20)
  {
    gx_offset += gx / 20;
    gy_offset += gy / 20;
    gz_offset += gz / 20;
    Serial.println(gx_offset);
  }
  
  else if(cpt == 20) { left(); }
  
  else if(cpt == 60) { stop(); }
  
  else {
    Serial.print ("Gyroscope readings:"); 
    Serial.print ("   \tGx:");
    Serial.print (gx - gx_offset);  
    Serial.print ("   \tGy:");
    Serial.print (gy - gy_offset);
    Serial.print ("   \tGz:");
    Serial.print (gz - gz_offset);  
    Serial.println ("\t");
    
  }
  
    // End of line
    delay(100); 
}








