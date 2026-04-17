
#include <Wire.h>   // digital inputs, sensor read-in
#include <VL53L1X.h>  // distance sensor control
#include <Servo.h>  // servo control
#include <math.h> // for trig

Servo myservo;
VL53L1X sensor3;


// Set Constants and create variables
const uint16_t PERIOD_MS = 20;
// float Dist3; 


float servo_center = 71;  //71
float servo_pos = 90;         // starting servo position
float MAX_SLEW = 90;


float theta;
float H0 = 185;  //134+60; //112;                 // beam height (mm) above sensor when at 0 degrees
float angle_sensor_arm = 290; //150;   // how far out from the axis we are measuring angle

float theta_set = 0 * PI/180;
float KA_p = 5.5;//4.5;//5;//0.75;  //0.05-0.075    0.12, 0.09
float KA_i = 0.6;//0.3;//0.01;               //Integral Gain    0.00175, 0.003
float KA_d = 0.35*KA_p;///0.25//0.4*KA_p;//0.5 ;  

float Error_A = 0;
float int_error_A = 0;            // Integral error start value
float prev_error_A = 0;          // 
float prev_der_error_A = 0;

float D_max = 400;


void setup() {
  Serial.begin(115200);
  // Serial.begin(230400);
    while (!Serial);  // wait for USB serial to connect
    delay(500);
    Serial.println("starting...");

  Serial1.begin(115200);

  myservo.attach(11);
  myservo.write(servo_center);

  // Wire.setClock(50000);  // lower to 50000 if hard to read from longer wires, default is 100000
  Wire.begin();

    // ---- Sensor 3 ---- Beam upwards
  // digitalWrite(XSHUT_3, HIGH);
  delay(10);
  // sensor3.init()
  if (!sensor3.init()) {
    Serial.println("sensor3 init failed!");
    while(1);  // stop here so you can see the message
  };
  sensor3.setDistanceMode(VL53L1X::Short);
  sensor3.setMeasurementTimingBudget(1000*PERIOD_MS); 
  sensor3.setROISize(8, 8);

  sensor3.startContinuous(PERIOD_MS);

  Serial.println(F("VL53L1X ready: Dist3, (Short mode)"));

}


void loop() {
  // timing - must be first
  static unsigned long t_last = 0;
  unsigned long t_now = micros();
  float dt = (t_now - t_last) / 1000000.0;
  if (dt > 0.5) dt = 0.5;  // clamp first loop
  t_last = t_now;

  // Read sensor 3
  float Dist3 = sensor3.read(PERIOD_MS);  // can drop careful time readout as this one doesn't interfere?
  if (sensor3.timeoutOccurred()) {
      Serial.println("sensor3 timeout!");
  }
  Dist3 = min(D_max,Dist3);


  // // Read theta_set from Mega
  if (Serial1.available()) {
      float received = Serial1.parseFloat();
      if (received > -1.0 && received < 1.0) {  // sanity check
          theta_set = received;
      }
  }
  // theta_set = -0;              // beam should hold level
  // theta_set = 5 * PI/180;     // ~5 degrees, small positive tilt
  // theta_set = 15 * PI/180;    // ~5 degrees opposite way

  float max_tilt = 15*PI/180;
  //if (abs(theta_set) > (max_tilt*PI/180)) {
  if (abs(theta_set) > (max_tilt)) {
    if (theta_set > 0) {
      theta_set = max_tilt;
    } else {
      theta_set = - max_tilt;
    }
  }

  // measure tilt angle theta from up-pointing dist sensor
  float opposite = H0 - Dist3;
  theta = -atan(opposite/angle_sensor_arm);

  // angle PID loop
  prev_error_A = Error_A;  // save for next loop
  Error_A = (theta - theta_set);
  // int_error_A = int_error_A + Error_A;        // update integral error 
  int_error_A = constrain(int_error_A + Error_A * dt, -250.0, 250.0);
  // float der_error = (Error - prev_error);       // note: 10 frame smoothing already done on T before error
  float der_error_A = 0.7*((Error_A - prev_error_A)/dt) + 0.3*prev_der_error_A;
  prev_der_error_A = der_error_A;

  float P_A = KA_p * Error_A;
  float I_A = KA_i * int_error_A;    // Integral 'effort'
  float D_A = KA_d * der_error_A;    // Derivative 'effort'
  float Effort_A = (180/PI)*(P_A + I_A + D_A); 



  float target = servo_center + Effort_A;
  servo_pos += constrain(target - servo_pos, -MAX_SLEW, MAX_SLEW);
  servo_pos = constrain(servo_pos, 0, 180);
  // servo_pos = 45;
  myservo.write(servo_pos);
  // myservo.write(servo_center+D_set/2);
  // myservo.write(servo_center);

  // serial monitor
  Serial.print(F("\tDist3:"));
  Serial.print(Dist3);
  Serial.print(F("\ttheta:"));
  Serial.print(theta*180/PI);

  Serial.print(F("\ttheta_set:"));
  Serial.print(theta_set*180/PI);
  Serial.print(F("\tP_A:"));
  Serial.print(P_A);
  Serial.print(F("\tI_A:"));
  Serial.print(I_A);
  Serial.print(F("\tD_A:"));
  Serial.print(D_A);
  Serial.print(F("\tE_A:"));
  Serial.print(Error_A);
  Serial.print(F("\tEffort_A:"));
  Serial.print(Effort_A);
  // Serial.println("");
  Serial.print(F("\tdt_ms:")); Serial.println(dt * 1000.0);
  t_last = t_now;

}
