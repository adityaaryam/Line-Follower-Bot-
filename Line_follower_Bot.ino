/*Initializing Motor Pins for Motor 1 */
int enA = 5;
int in1 = 7;
int in2 = 6;
/* Initializing Motor Pins for Motor 2 */
int enB = 10;
int in3 = 8;
int in4 = 9;
/* Note enA and enB must be connected to the PWM pins of the arduino */

/* Initialize Base-speed and Maxspeed for both the motors */
const uint8_t maxspeed_A = 200;
const uint8_t maxspeed_B = 200;
const uint8_t basespeed_A = 130;
const uint8_t basespeed_B = 130;

/* Initailizng the Working Variables */
float elapsedTime, current_time, timePrev;
float pid, err, previous_error = 0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/*Setting up the P, I, D parameters of the PID Controller which has been programmed further*/
double kp = 10;
double ki = 0.02;
double kd = 0.2;

int speed_A;
int speed_B;

void PID(int error)
{
  /*How long since we last calculated*/
  current_time = millis();
  elapsedTime = (current_time - timePrev) / 1000;

  /*Compute all the working error variables*/
  pid_p = error;
  pid_i = pid_i + (error * elapsedTime);
  pid_d = ((error - previous_error) / elapsedTime);

  /*Compute PID Output*/
  pid = (kp * pid_p) + (ki * pid_i) + (kd * pid_d);

  /* Assign the motor speed after receiving the pid speed */
  speed_A = basespeed_A + pid;
  speed_B = basespeed_B - pid;

  if (speed_A > maxspeed_A) {     /*If the speed exceeds the maxspeed, it is saturated to the max-speed. This ensures that the motors doesn't get damagaed */ 
    speed_A = maxspeed_A;
  }
  if (speed_B > maxspeed_B) {
    speed_B = maxspeed_B;
  }

  if (speed_A < 0) {
    speed_A = 0;
  }
  if (speed_B < 0) {
    speed_B = 0;
  }

  /*Remember some variables for next time*/
  previous_error = error;
  timePrev = current_time;
}

void setup() {
  /* All the arduino pins used are outputs */
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);

  current_time = millis();
  /* Begin the Serial Communication */
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void moveForward(int speed1, int speed2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed1); /* Speed Range 0-255 */
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed2);
}

void loop() {

  while (!Serial.available());
  int errors = Serial.readString().toInt();   /* Reading error from the Serial Port which has been sent as a result of Path Detection */
  PID(errors);
  moveForward(speed_A, speed_B);
}
