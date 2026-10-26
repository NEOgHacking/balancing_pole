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
#define EN 3

// Define the BLDC motor itself
BLDCMotor motor = BLDCMotor(7);

// Define the motor driver used
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM1, PWM2, PWM3, EN);

// Define the magnetic encoder on the back of the motor
MagneticSensorSPI encoder = MagneticSensorSPI(AS5048_SPI, CSn);
// And the encoder to see the angle of the linkage
MagneticSensorI2C pendulum = MagneticSensorI2C(0x36, 12, 0x0E, 4);

void setup() {
  
  // initialise motor encoder hardware
  encoder.init();
  
  // init the pendulum encoder
  pendulum.init();
  
  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 5;

  // link the motor to the encoder
  motor.linkSensor(&encoder);
  
  // driver
  driver.voltage_power_supply = 12; 
  driver.init();
  // link the driver and the motor
  motor.linkDriver(&driver);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
}

// loop downsampling counter
long loop_count = 0;
float target_voltage;

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // if angle controllable
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - x = [pendulum angle, pendulum velocity, motor velocity]' 
  float u =  40*p_angle + 7*p_vel + 0.3*m_vel;
  
  // limit the voltage set to the motor
  if(abs(u) > motor.voltage_limit*0.7) u = _sign(u)*motor.voltage_limit*0.7;
  
  return u;
}

void loop() {
  // ~1ms 
  motor.loopFOC();

  // control loop each ~25ms
  if(loop_count++ > 25){
    // updating the pendulum angle sensor
    // NECESSARY for library versions > v2.2 
    pendulum.update();
    
    // calculate the pendulum angle 
    float pendulum_angle = constrainAngle(pendulum.getAngle() + 1.97);

    if( abs(pendulum_angle) < 0.5 ) // if angle small enough stabilize
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());
    else // else do swing-up
      // sets 40% of the maximal voltage to the motor in order to swing up
      target_voltage = -_sign(pendulum.getVelocity())*motor.voltage_limit*0.4;

    // set the target voltage to the motor
    motor.move(target_voltage);
    Serial.println(target_voltage);

    // restart the counter
    loop_count=0;
  }
   

}