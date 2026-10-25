#include <Arduino.h>
#include <SimpleFOC.h>


// The pinout for the encoder on the back of the GM2804 motor
// VDD = Red
// GND = White
#define CSn 7 // Black cable
#define CLK 8 // Purple cable
#define MISO 9 // Green cable
#define MOSI 10 // Yellow cable

// Define the motor driver pins here
#define PWM1 0
#define PWM2 1
#define PWM3 2

// Define the BLDC motor itself
BLDCMotor motor = BLDCMotor(7);

// Define the motor driver used
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM1, PWM2, PWM3);

// Define the magnetic encoder on the back of the motor
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, CSn);
// And the encoder to see the angle of the linkage
MagneticSensorI2C sensor_link = MagneticSensorI2C(AS5600_I2C);


// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }


void setup() {
  Serial.begin(115200);

  // Initate the sensor
  sensor.init();
  // Link it to the motor
  motor.linkSensor(&sensor);

  // Initate the driver
  driver.init();
  // Link it to the motor
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 3;

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  motor.useMonitoring(Serial);

  if(!motor.init()) {
    Serial.println("Motor initation failed");
    return;
  }

  if(motor.initFOC()) {
    Serial.println("FOC initation failed");
    return;
  }

  motor.target = 3;

  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
}