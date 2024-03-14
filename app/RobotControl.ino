/*
Arduino Contolled Robotarm

Control the robot with a small potentiometer miniature or an android app.
Android app tested on android 11, 13 and 14. Doesn't run on all android 13 devices.

Manual control active if no Bluetooth-device connected, indicated by Mode-LED.

*/

#include <Wire.h>                               // I2C Communication
#include <Adafruit_PWMServoDriver.h>            // Servo Driver
#include <ArduinoBLE.h>                         // Bluetooth Low Energy

// PWM Settings
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Assign servoaddresses to pins on servoboard
int gripper = 11;
int wrist = 12;
int shoulder = 15;
int base = 14;
int elbow = 13;

//Assign potentiometers to pins on Arduino Uno
int potWrist = A0;
int potElbow = A1;  
int potShoulder = A2;
int potBase = A3;

int wrist_pos, elbow_pos, shoulder_pos, base_pos, gripper_pos;                 // current position
int wrist_SP[50], elbow_SP[50], shoulder_SP[50], base_SP[50], gripper_SP[50];  // for storing positions/steps
int index, stop = 0;                                                           // variables for RUN-Mode

BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");  // create the Bluetooth Low Energy Robot Service

// Bluetooth Low Energy Characteristics - custom 128-bit UUID, read and writable by central
BLEIntCharacteristic gripperCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic wristCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic elbowCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic shoulderCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic baseCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEIntCharacteristic stopCharacteristic("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic saveCharacteristic("19B10007-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic runCharacteristic("19B10008-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic resetCharacteristic("19B10009-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {

  Serial.begin(9600);
  while (!Serial)
    ;
    
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("ROBOT");
  BLE.setAdvertisedService(robotService);

  // add the characteristics to the service
  robotService.addCharacteristic(gripperCharacteristic);
  robotService.addCharacteristic(wristCharacteristic);
  robotService.addCharacteristic(elbowCharacteristic);
  robotService.addCharacteristic(shoulderCharacteristic);
  robotService.addCharacteristic(baseCharacteristic);
  robotService.addCharacteristic(saveCharacteristic);
  robotService.addCharacteristic(runCharacteristic);
  robotService.addCharacteristic(resetCharacteristic);
  robotService.addCharacteristic(stopCharacteristic);

  // add service
  BLE.addService(robotService);

  // set the initial value for the positions and characeristics:
  gripper_pos = 90;
  wrist_pos = 300;
  elbow_pos = 545;
  shoulder_pos = 90;
  base_pos = 340;

  gripperCharacteristic.writeValue(gripper_pos);
  wristCharacteristic.writeValue(wrist_pos);
  elbowCharacteristic.writeValue(elbow_pos);
  shoulderCharacteristic.writeValue(shoulder_pos);
  baseCharacteristic.writeValue(base_pos);
  saveCharacteristic.writeValue(0);
  runCharacteristic.writeValue(0);
  resetCharacteristic.writeValue(0);
  stopCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE ROBOT Peripheral");

  // Move the Servos to Home-Position
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(gripper, 0, gripper_pos);
  pwm.setPWM(wrist, 0, wrist_pos);
  pwm.setPWM(elbow, 0, elbow_pos);
  pwm.setPWM(shoulder, 0, shoulder_pos);
  pwm.setPWM(base, 0, base_pos);

  delay(2000);

  // Pinmode for gripper Pushbutton and Mode-LED, Mode-LED indicates if Manual-Mode is active
  pinMode(13, INPUT_PULLUP);
  pinMode(12, OUTPUT);
}

// Function for Controling the servos with the Potentiometers when Manual-Mode active
void moveMotor(int controlIn, int motorOut) {
  int pulse_wide, pulse_width, potVal;

  potVal = analogRead(controlIn);                                         //Read value of Potentiometer
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);   //Map Potentiometer position to Motor
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);      //Calculatethe Pulsewidth, Pulswidth = angle 

  pwm.setPWM(motorOut, 0, pulse_width);
}


void loop() {
  // listen for Bluetooth Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    digitalWrite(12, LOW);                              //Turn of Mode-LED, App-Control is active
    Serial.print("Connected to central: ");

    // print the central's MAC address:
    Serial.println(central.address());
    Serial.println("Activated App-Control");

    // while the central is still connected to peripheral:
    while (central.connected()) {

      // if a value has changed, read the value and move the servo to that position
      if (gripperCharacteristic.written()) {
        gripper_pos = gripperCharacteristic.value();
        pwm.setPWM(gripper, 0, gripper_pos);
      }

      if (wristCharacteristic.written()) {
        wrist_pos = wristCharacteristic.value();
        pwm.setPWM(wrist, 0, wrist_pos);
      }

      if (elbowCharacteristic.written()) {
        elbow_pos = elbowCharacteristic.value();
        pwm.setPWM(elbow, 0, elbow_pos);
      }

      if (shoulderCharacteristic.written()) {
        shoulder_pos = shoulderCharacteristic.value();
        pwm.setPWM(shoulder, 0, shoulder_pos);
      }

      if (baseCharacteristic.written()) {
        base_pos = baseCharacteristic.value();
        pwm.setPWM(base, 0, base_pos);
      }

      // store the current position in the array
      if (saveCharacteristic.written()) {
        gripper_SP[index] = gripper_pos;
        wrist_SP[index] = wrist_pos;
        elbow_SP[index] = elbow_pos;
        shoulder_SP[index] = shoulder_pos;
        base_SP[index] = base_pos;
        index++;
      }

      // if RUN pressed, move to  every position stored until stop is pressed
      if (runCharacteristic.written()) {

        if (stop = 1) {                           //if stop ist still 1 from a previous run set it to 0
          stopCharacteristic.writeValue(0);               
          stop = 0;
        }

        while (stop != 1) {

          for (int i = 0; i <= index; i++) {

            pwm.setPWM(gripper, 0, gripper_SP[i]);
            pwm.setPWM(wrist, 0, wrist_SP[i]);
            pwm.setPWM(elbow, 0, elbow_SP[i]);
            pwm.setPWM(shoulder, 0, shoulder_SP[i]);
            pwm.setPWM(base, 0, base_SP[i]);
            delay(400);

            BLEDevice central = BLE.central();      //keep the BLE Connection alive, without this line the app will crash (Connection Error)
            stop = stopCharacteristic.value();   
          }
        }
      }

      // clear the Positions array when reset pressed
      if (resetCharacteristic.written()) {
        memset(gripper_SP, 0, sizeof(gripper_SP));
        memset(wrist_SP, 0, sizeof(wrist_SP));
        memset(elbow_SP, 0, sizeof(elbow_SP));
        memset(shoulder_SP, 0, sizeof(shoulder_SP));
        memset(base_SP, 0, sizeof(base_SP));
        index = 0;
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());

  } else if (!central) {
    // if no devive connected activate the manual controls with the potentiometers
    digitalWrite(12, HIGH);
    Serial.println("No Device connected");
    Serial.println("Activated Manual-Control");

    //Assign Motors to corresponding Potentiometers
    moveMotor(potWrist, wrist);
    moveMotor(potElbow, elbow);
    moveMotor(potShoulder, shoulder);
    moveMotor(potBase, base);

    int pushButton = digitalRead(13);
    if (pushButton == LOW) {
      pwm.setPWM(gripper, 0, 180);    //Keep Gripper closed when button is not pressed
    } else {
      pwm.setPWM(gripper, 0, 90);     //Open Gripper when button is pressed
    }
  }
}
