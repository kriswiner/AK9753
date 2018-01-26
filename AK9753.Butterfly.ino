/* 
 * 01/26/2018 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The AK9753 is a 2 x 2 IR I2C sensor array useful for presence and motion detection.
 *  
 *  This sketch uses default SDA/SCL pins on the Dragonfly/Butterfly development boards
 *
 *  Library may be used freely and without limit with attribution.
  */
  
  #include <Wire.h>

#define AK9753_WIA1    0x00
#define AK9753_WIA2    0x01
#define AK9753_INFO1   0x02
#define AK9753_INFO2   0x03
#define AK9753_INTST   0x04
#define AK9753_ST1     0x05
#define AK9753_IR1L    0x06
#define AK9753_IR1H    0x07
#define AK9753_IR2L    0x08
#define AK9753_IR2H    0x09
#define AK9753_IR3L    0x0A
#define AK9753_IR3H    0x0B
#define AK9753_IR4L    0x0C
#define AK9753_IR4H    0x0D
#define AK9753_TMPL    0x0E
#define AK9753_TMPH    0x0F
#define AK9753_ST2     0x10
#define AK9753_EINTEN  0x1B
#define AK9753_ECNTL1  0x1C
#define AK9753_CNTL2   0x1D

#define AK9753_ADDRESS 0x64 // when CAD0 = CAD1 = LOW (default)

#define pdownPin 9
#define intPin   8
#define myLed1  13
#define myLed2  26
#define myLed3  38

#define standby     0x00
#define singleShot  0x02
#define continuous0 0x04  // 100% duty cycle
#define continuous1 0x05  //  50% duty cycle
#define continuous2 0x06  //  25% duty cycle
#define continuous3 0x07  //  12.5% duty cycle

#define bw_0_3Hz 0x00  // 0.576 s period in contiuous mode
#define bw_0_6Hz 0x01  // 0.276 s period
#define bw_1_1Hz 0x02  // 0.144 s period
#define bw_2_2Hz 0x03  // 2.2 Hz, 0.072 s period
#define bw_4_4Hz 0x04  // 0.036 s period
#define bw_8_8Hz 0x05  // 0.018 s period

bool newData = false;
int16_t AK9753Data[5] = {0, 0, 0, 0, 0};
float IRscale =  14286.4f/32767.0f;
float Tscale =  1.0f/512.0f;
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
float IR[4] = {0., 0., 0., 0.};
uint32_t dataCount = 0;
float topdown = 0., leftright = 0., topdownavg = 0., leftrightavg = 0.;
float runtopdownavg = 0., runleftrightavg = 0.;
uint32_t runCount = 1;

uint8_t MODE = continuous0, BW = bw_2_2Hz; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, HIGH);  // active LOW

  pinMode(myLed2, OUTPUT);
  digitalWrite(myLed2, HIGH);  // active LOW

  pinMode(myLed3, OUTPUT);
  digitalWrite(myLed3, HIGH);  // active LOW

  pinMode(intPin, INPUT);
  
  pinMode(pdownPin, OUTPUT);
  digitalWrite(pdownPin, HIGH); // power up the AK9753
  delay(100);
  
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  I2Cscan();

  // Read device code information as a test of I2C communications
  uint8_t c = readByte(AK9753_ADDRESS, AK9753_WIA1);
  Serial.print("Company code is 0x"); Serial.print(c, HEX), Serial.println(" should be 0x48");
  uint8_t d = readByte(AK9753_ADDRESS, AK9753_WIA2);
  Serial.print("Device ID is 0x"); Serial.print(d, HEX), Serial.println(" should be 0x13");

  if(c == 0x48 && d == 0x13)
  {
    Serial.println("AK9753 is online!");
    
    Reset(); //Soft reset the AK9753

    Init();  // Initialize AK9753

    attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for INT pin output of AK9753
    readByte(AK9753_ADDRESS, AK9753_ST2);  // clear interrupt
    
  }
  else
  {
    Serial.print("Could not connect to AK9753: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
/* end of setup */
}


void loop() {
   // If intPin goes high, all data registers have new data
   if(newData == true) {  // On interrupt, read data
     newData = false;  // reset newData flag

     if(readByte(AK9753_ADDRESS, AK9753_INTST) & 0x01) {  // verify interrupt due to data ready
     readData(AK9753Data);
     dataCount++;
     }
     
     IR[0] = (float) AK9753Data[0] * IRscale;  // convert raw data into picoAmps
     IR[1] = (float) AK9753Data[1] * IRscale;
     IR[2] = (float) AK9753Data[2] * IRscale;
     IR[3] = (float) AK9753Data[3] * IRscale;

     topdown += IR[2] - IR[0];
     leftright += IR[3] - IR[1];
   }


    // Serial print and/or display at 1 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update serial once per half-second independent of read rate
    runCount++;
    
    topdownavg = (topdown/(float) dataCount);
    leftrightavg = (leftright/(float) dataCount);

    Serial.print("topdownavg = "); Serial.println(topdownavg, 2);
    Serial.print("leftrightavg = "); Serial.println(leftrightavg, 2);
    
    if( IR[3] > 0 && IR[0] > 0 && (IR[3] - IR[1]) > 400.) 
    {
      Serial.println("left to right");
      digitalWrite(myLed2, LOW); delay(10); digitalWrite(myLed2, HIGH);
    }
    if( IR[3] > 0 && IR[0] > 0 && (IR[3] - IR[1]) < 200.) 
    {
      Serial.println("right to left");
      digitalWrite(myLed3, LOW); delay(10); digitalWrite(myLed3, HIGH);
    }

     Serial.print("IR1 = "); Serial.print((float) AK9753Data[0] * IRscale, 2); Serial.println(" pA");
     Serial.print("IR2 = "); Serial.print((float) AK9753Data[1] * IRscale, 2); Serial.println(" pA");
     Serial.print("IR3 = "); Serial.print((float) AK9753Data[2] * IRscale, 2); Serial.println(" pA");
     Serial.print("IR4 = "); Serial.print((float) AK9753Data[3] * IRscale, 2); Serial.println(" pA");
     Serial.print("Temp = "); Serial.print((float)AK9753Data[4] * Tscale + 26.75f, 2); Serial.println(" degrees C");

//    digitalWrite(myLed1, !digitalRead(myLed1));
    count = millis(); 
    topdown = 0;
    leftright = 0;
    dataCount = 0;
    }

}

/* Useful functions */

void myinthandler()
{
  newData = true;
}

void Reset()
{
  uint8_t temp = readByte(AK9753_ADDRESS, AK9753_CNTL2);
  writeByte(AK9753_ADDRESS, AK9753_CNTL2, temp | 0x01);
}

void Init()
{
  // initialize AK9753
  uint8_t temp = readByte(AK9753_ADDRESS, AK9753_EINTEN);
  writeByte(AK9753_ADDRESS, AK9753_EINTEN, temp | 0x01); // enable data ready interrupt
  writeByte(AK9753_ADDRESS, AK9753_ECNTL1, MODE |  BW << 3 ); 
}

void readData(int16_t * destination)
{
  uint8_t rawData[11];  // x/y/z accel register data stored here
  if(readByte(AK9753_ADDRESS, AK9753_ST1) & 0x01)  // only read data if data ready == 1
  {
  readBytes(AK9753_ADDRESS, AK9753_IR1L, 11, &rawData[0]);  // Read the 11 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;   
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
  }
}


// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
      error = Wire.transfer(address, NULL, 0, NULL, 0);
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

// I2C read/write functions for the MPU9250 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
