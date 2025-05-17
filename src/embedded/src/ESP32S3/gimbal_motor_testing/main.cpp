// #include <SimpleFOC.h>
// #include <Wire.h>
// #include <math.h>
// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h>

// const char* ssid = "LIVINGBasics_Smart_Temp_Kettle";
// const char* password = "oldHag1984";

// // BLDC motor & driver instance
// // BLDCMotor motor = BLDCMotor(pole pair number);
// BLDCMotor motor = BLDCMotor(7);
// BLDCMotor motor2 = BLDCMotor(7);

// // BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
// // BLDCDriver3PWM driver = BLDCDriver3PWM(33,32,25,12);
// BLDCDriver3PWM driver = BLDCDriver3PWM(7,8,9);
// BLDCDriver3PWM driver2 = BLDCDriver3PWM(1,2,3);

// //TO RE-FLASH ESP32-S3: esptool.py --chip esp32s3 --port /dev/ttyACM0 erase_flash


// MagneticSensorI2C magnetic_encoder = MagneticSensorI2C(AS5600_I2C);


// // Stepper motor & driver instance



// //target variable
// float target_velocity = 0;
// float target_angle = 0;

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { 
//   // command.scalar(&target_velocity, cmd);
//   // motor.controller = MotionControlType::velocity_openloop; 
//   target_velocity = target_velocity;
// }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
// void doMotor(char* cmd) { command.motor(&motor, cmd); }
// void doAngleTarget(char* cmd) {
//   // command.scalar(&target_angle, cmd);
//   // motor.controller = MotionControlType::angle;
//   // target_velocity = target_angle;
// }




// void setup() {

//   // use monitoring with serial 
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);
//   WiFi.setHostname("ESP32-BLDC");

//   WiFi.begin(ssid, password);



//   while (WiFi.status() != WL_CONNECTED) {
//     delay(100);
//     Serial.println("CONNECTING");
//   }
//   Serial.println("CONNECTED");

//   ArduinoOTA.onStart([]() {
//     String type;
//     if (ArduinoOTA.getCommand() == U_FLASH) {
//       type = "sketch";
//     } else { 

//           type = "filesystem";
//     }
//     Serial.println("Start updating " + type);
//   });

//   ArduinoOTA.begin();
//   Serial.println("Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());



//   motor.useMonitoring(Serial);
  
//   motor2.useMonitoring(Serial);

  
//   // Serial.println("CHECKING FOR ADDRESS");

//   // for (byte i=0;i<127;i++) {
//   //   Wire.beginTransmission(i);
//   //   if (Wire.endTransmission()) {
//   //     Serial.println(i, HEX);
//   //   }
//   // }


//   motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
//   motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_VOLT_D | _MON_VOLT_Q | _MON_CURR_D;


//   // enable more verbose output for debugging
//   // comment out if not needed
//   SimpleFOCDebug::enable(NULL);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 12;
//   driver2.voltage_power_supply = 12;
//   // limit the maximal dc voltage the driver can set
//   // as a protection measure for the low-resistance motors
//   // this value is fixed on startup
//   driver.voltage_limit = 6;
//   driver2.voltage_limit = 6;

//   if(!driver.init() || !driver2.init()){
//     Serial.println("Driver init failed!");
//     return;
//   }
//   // link the motor and the driver
//   magnetic_encoder.init();

//   motor.linkDriver(&driver);
//   motor.linkSensor(&magnetic_encoder);


//   motor2.linkDriver(&driver2);

//   //motor.linkSensor(&magnetic_encoder);
//   // limiting motor movements
//   // limit the voltage to be set to the motor
//   // start very low for high resistance motors
//   // current = voltage / resistance, so try to be well under 1Amp
//   motor.voltage_limit = 4;   // [V]
//   motor.current_limit = 0.2;
 
//   // open loop control config
//   motor.controller = MotionControlType::velocity_openloop;

//   motor2.voltage_limit = 4;   // [V]
//   motor2.current_limit = 0.2;
 

//   //motor.initFOC();
  

//   // open loop control config
//   motor2.controller = MotionControlType::velocity_openloop;

//   // init motor hardware
//   if(!motor.init() || !motor2.init()){
//     Serial.println("Motor init failed!");
//     return;
//   }
  

//   // add target command T
//   command.add('T', doTarget, "target velocity");
//   command.add('L', doLimit, "voltage limit");
//   command.add('M',doMotor,"motor");
//   command.add('A',doAngleTarget,"angle");


//   Serial.println("Motor ready!");


//   Serial.println("Set target velocity [rad/s]");


//   // Serial.println(motor.enabled);
// }

// void loop() {


//   // open loop velocity movement
//   // using motor.voltage_limit and motor.velocity_limit
//   // to turn the motor "backwards", just set a negative target_velocity

 
 
//   // Serial.println("AAA");
 
//   //analogWrite(26, 128); // 50% duty cycle (speed control)
//   // delay(5000); // Run for 5 seconds

//  //Serial.println(magnetic_encoder.getAngle());

//   ArduinoOTA.handle();

//   motor.loopFOC();
//   motor2.loopFOC();

//   motor.move(target_velocity);
//   motor2.move(target_velocity);

//   motor.monitor();

//   //Serial.println(magnetic_encoder.getVelocity());


//   // analogWrite(26, 0); // Stop the motor
//   // delay(5000); // Wait 5 seconds

//   // user communication
//   command.run();

// }

// Open loop motor control example
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(8,9,7);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);


//target variable
float target_velocity = 1;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motor.move(target_velocity);

  // user communication
  command.run();
}


