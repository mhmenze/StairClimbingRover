TaskHandle_t Task1;
//TaskHandle_t Task2;

#include "BluetoothSerial.h" //Header File for Serial Bluetooth
BluetoothSerial ESP_BT; //Object for Bluetooth
#include <Wire.h>

#include "I2Cdev.h"
#include <ESP32Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL 
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}
 
int  servoPin2  =  25 , servoPin3  =  26 ;
Servo servo2, servo3;

String incoming;// varible store received data

const int BST_LPwm = 27;    // I/O channel setup ESP32 pin 
const int BST_RPwm = 12;    // I/O chennel setup ESP32 pin
const int En_LR    = 14;    // I/O pin for BST l_EN & R_EN (enable)
int Speed = 255 ;

const int  Channel_15 = 15;   // & PWM channel 0, for BST pin L_PWM 
const int  Channel_14 = 14;   // & PWM channel 1, for BST pin R_PWM 

const int freq = 1000;      //  Set up PWM Frequency
const int  res = 8;         //  Set up PWM Resolution 

void setup() {

  ledcSetup(Channel_15, freq,res); // setup PWM channel for BST L_PWM
  ledcSetup(Channel_14, freq,res); // setup PWM channel for BST R_PWM
  ledcAttachPin( BST_LPwm , Channel_15); // Attach BST L_PWM
  ledcAttachPin( BST_RPwm , Channel_14); // Attach BST R_PWM
  pinMode(En_LR ,OUTPUT); // declare pin as output 

  servo2.attach (servoPin2);
  servo3.attach (servoPin3);

  servo2.write ( 0 ); // Init the servo2 angle to 0
  servo3.write ( 0 ); // Init the servo2 angle to 0

  Serial.begin(115200); 
  Wire.begin(21, 22, 100000); // sda, scl, clock speed
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
 
  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(37);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(85);

    mpu.setXAccelOffset(-4386); // 1688 factory default for my test chip
    mpu.setYAccelOffset(-343); 
    mpu.setZAccelOffset(1316); 
 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
  }
 

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "RC_Car",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

}

//Task1code: Bluetooth RC Car
void Task1code( void * pvParameters ){
  ESP_BT.begin("Stair Climber"); //Name of your Bluetooth Signal
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println("Bluetooth Device is Ready to Pair");

  for(;;){
    digitalWrite(En_LR,HIGH); // enable BST  
    
      if (ESP_BT.available()) //Check if we receive anything from Bluetooth
      {
      incoming = char(ESP_BT.read()); //Read what we received
      }
      Serial.println(incoming);
      if (incoming == "F") { // forward     
       ledcWrite(Channel_15,0);       // stop channel 14  (R_PWM)     
       ledcWrite(Channel_14,Speed);   // run channel 15 (L_PWM) at "speed"
      }
      else if (incoming == "B") { // reverse
       ledcWrite(Channel_14,0);       // stop channel 15 (L_PWM)
       ledcWrite(Channel_15,Speed);   // run channel 14 (R_PWM) at "speed"
      }
//      else if (incoming == "R") { //turn right
//      Serial.println("Right");
//      }
//      else if (incoming == "L") { // turn left
//      Serial.println("Left");
//      }
//      
      else if (incoming == "S") { // turn stop
       ledcWrite(Channel_14,0);       // stop channel 14  (R_PWM)
       ledcWrite(Channel_15,0);       // stop chennel 15  (L_PWM)
      }
      
      else {
       ledcWrite(Channel_14,0);       // stop channel 14  (R_PWM)
       ledcWrite(Channel_15,0);       // stop chennel 15  (L_PWM)
      }
      vTaskDelay(2); //Very Very Important!
  }
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
 
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
 
        #ifdef  OUTPUT_READABLE_YAWPITCHROLL
 
            // display Euler angles in degrees
 
            mpu.dmpGetQuaternion (& q, fifoBuffer);
            mpu.dmpGetGravity (& gravity, & q);
            mpu.dmpGetYawPitchRoll (ypr, & q, & gravity);
 
            Serial.print ( "ypr \ t" );
            Serial.print (ypr [ 0 ]  *  180 / M_PI);
 
            Serial.print ( "\ t" );
            Serial.print (ypr [ 1 ]  *  180 / M_PI);            
            servo2.write (map (ypr [ 1 ]  *  180 / M_PI,  - 90 ,  90 ,  0 ,  180 )); // Control servo2
 
            Serial.print ( "\ t" );
            Serial.println (ypr [ 2 ]  *  180 / M_PI);
            servo3.write (map (ypr [ 2 ]  *  180 / M_PI,  90 ,  -90 ,  0 ,  180 )); // Control servo3       
 
        #endif
    }
}
