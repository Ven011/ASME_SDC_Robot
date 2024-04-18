#include <Servo.h>
#include <Encoder.h>

/* Receiver joystick pins and settings */
#define FB_JOY          2 
#define LR_JOY          3
#define THROTTLE_JOY    4
#define MODE_SWITCH     9
#define PUTTER_SWITCH   10

#define JOY_MIN 980
#define JOY_MAX 1986
#define JOY_AVG ((JOY_MIN + JOY_MAX)/2)

/* Setup left motors */
#define LEFT_SPEED_PWM_1 5
#define LEFT_SPEED_PWM_2 6

#define LEFT_IN1 38
#define LEFT_IN2 40
#define LEFT_IN3 42
#define LEFT_IN4 44

/* Right motors speed pins and  */
#define RIGHT_SPEED_PWM_1 7
#define RIGHT_SPEED_PWM_2 8

#define RIGHT_IN1 46
#define RIGHT_IN2 48
#define RIGHT_IN3 50
#define RIGHT_IN4 52

/* Servo input pins */
#define LIFTER_SERVO1 11
#define LIFTER_SERVO2 12

/* direction macros */
#define STOP          0
#define FORWARD       1
#define BACKWARD      2
#define LEFT          3
#define RIGHT         4
#define STOP_DEADZONE 40

/* putter motors and settings*/
#define PUTTER_SPEED_PWM  13
#define CW                0
#define CCW               1
#define NO_DIR            2
#define LEFT_ENCODER_A    18
#define LEFT_ENCODER_B    19
#define RIGHT_ENCODER_A   20
#define RIGHT_ENCODER_B   21

#define PUTT_IN1 36
#define PUTT_IN2 34
#define PUTT_IN3 32
#define PUTT_IN4 30

/* servo setup */
Servo servo1;
Servo servo2;

/* encoder setup */
Encoder leftENC(LEFT_ENCODER_A, LEFT_ENCODER_B);
Encoder rightENC(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

int init_left_enc_pos = 0;
int init_right_enc_pos = 0;

void drive();
void lifter();
void mode_manager();
void putt();
void putter_mode();
void drive_mode();
void robot_off();

void setup() 
{
  Serial.begin(9600);

  /* setup receiver pins */
  pinMode(MODE_SWITCH, INPUT);
  
  /* Setup motors pins*/
  pinMode(FB_JOY, INPUT);
  pinMode(LR_JOY, INPUT);
  pinMode(LEFT_SPEED_PWM_1, OUTPUT);
  pinMode(LEFT_SPEED_PWM_2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  pinMode(RIGHT_SPEED_PWM_1, OUTPUT);
  pinMode(RIGHT_SPEED_PWM_2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  /* setup putter pins */
  pinMode(PUTTER_SPEED_PWM, OUTPUT);
  pinMode(PUTT_IN1, OUTPUT);
  pinMode(PUTT_IN2, OUTPUT);
  pinMode(PUTT_IN3, OUTPUT);
  pinMode(PUTT_IN4, OUTPUT);

  /* Setup servos */
  servo1.attach(LIFTER_SERVO1, 500, 2400);
  servo2.attach(LIFTER_SERVO2, 500, 2400);

}

void loop() 
{
  mode_manager();
}

void robot_off()
{
  // set motor speed to zero
  analogWrite(LEFT_SPEED_PWM_1, 0);
  analogWrite(LEFT_SPEED_PWM_2, 0);
  analogWrite(RIGHT_SPEED_PWM_1, 0);
  analogWrite(RIGHT_SPEED_PWM_2, 0);
}

void drive()
{
  // use servo mode switcher as an on/off switch
  int on_off = pulseIn(PUTTER_SWITCH, HIGH);
  if(on_off > JOY_AVG)
  {
    robot_off();
    while(pulseIn(PUTTER_SWITCH, HIGH) > JOY_AVG);
  } 
  
  // direction
  int move_dir = STOP;
  
  // Read the joystick input
  int fb_input = pulseIn(FB_JOY, HIGH);
  int lr_input = pulseIn(LR_JOY, HIGH);

  // translate input to speed between -255 -> 255
  int y = map(fb_input, JOY_MIN, JOY_MAX, -255, 255);
  int x = map(lr_input, JOY_MIN, JOY_MAX, -255, 255);

  // determine movement direction
  if(y>STOP_DEADZONE && y>x && y>-x)
    move_dir = RIGHT;
  else if(y<-STOP_DEADZONE && y<-x && y<x)
    move_dir = LEFT;
  else if(x<-STOP_DEADZONE && y<-x && y>x)
    move_dir = FORWARD;
  else if(x>STOP_DEADZONE && y>-x && y<x)
    move_dir = BACKWARD;

  // determine the speed as the magnitude of the forward and backward inputs
  int move_speed = sqrt(pow(y, 2) + pow(x, 2));

  if(move_speed > 200)
    move_speed = 255;

  // set motor speed
  analogWrite(LEFT_SPEED_PWM_1, move_speed);
  analogWrite(LEFT_SPEED_PWM_2, move_speed);
  analogWrite(RIGHT_SPEED_PWM_1, move_speed);
  analogWrite(RIGHT_SPEED_PWM_2, move_speed);

  // set motor direction
  switch(move_dir)
  {
    case FORWARD:
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
      digitalWrite(LEFT_IN3, HIGH);
      digitalWrite(LEFT_IN4, LOW);

      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
      digitalWrite(RIGHT_IN3, HIGH);
      digitalWrite(RIGHT_IN4, LOW);
      break;
    case BACKWARD:
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
      digitalWrite(LEFT_IN3, LOW);
      digitalWrite(LEFT_IN4, HIGH);

      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
      digitalWrite(RIGHT_IN3, LOW);
      digitalWrite(RIGHT_IN4, HIGH);
      break;
    case RIGHT:
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
      digitalWrite(LEFT_IN3, LOW);
      digitalWrite(LEFT_IN4, HIGH);

      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
      digitalWrite(RIGHT_IN3, HIGH);
      digitalWrite(RIGHT_IN4, LOW);
      break;
    case LEFT:
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
      digitalWrite(LEFT_IN3, HIGH);
      digitalWrite(LEFT_IN4, LOW);

      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
      digitalWrite(RIGHT_IN3, LOW);
      digitalWrite(RIGHT_IN4, HIGH);
      break;
  }
}

void lifter()
{
  // read receiver input
  int lifter_input = pulseIn(THROTTLE_JOY, HIGH);

  // translate input to servo position
  int servo1_input = map(lifter_input, 980, 1990, 0, 180);
  int servo2_input = map(lifter_input, 980, 1990, 180, 0);

  // set servo positions
  servo1.write(servo1_input);
  servo2.write(servo2_input);
}

void putt()
{
  // check putter selection switch
  int putt_select = pulseIn(PUTTER_SWITCH, HIGH);

  // set putter rotation speed based on throttle position
  int putt_speed = pulseIn(FB_JOY, HIGH);
  Serial.println(putt_speed);
  
  putt_speed = map(putt_speed, JOY_MAX, JOY_MIN, -255, 255);
  
  if(putt_speed > 200)
    putt_speed = 255;
  else if(putt_speed < -200)
    putt_speed = -255;

  analogWrite(PUTTER_SPEED_PWM, abs(putt_speed));
  uint8_t motor_move_dir;

  // determine rotation direction
  if(putt_speed > 0)
    motor_move_dir = CW;
  else if(putt_speed < 0)
    motor_move_dir = CCW;
  else
    motor_move_dir = NO_DIR;

  // select putter based on receiver output
  if(putt_select >= JOY_AVG) // Left putter
  {
    // move motor
    switch(motor_move_dir)
    {
      case CW:
        digitalWrite(PUTT_IN1, HIGH);
        digitalWrite(PUTT_IN2, LOW);
        digitalWrite(PUTT_IN3, LOW);
        digitalWrite(PUTT_IN4, LOW);
        break;
      case CCW:
        digitalWrite(PUTT_IN1, LOW);
        digitalWrite(PUTT_IN2, HIGH);
        digitalWrite(PUTT_IN3, LOW);
        digitalWrite(PUTT_IN4, LOW);
        break;
      case NO_DIR:
        digitalWrite(PUTT_IN1, HIGH);
        digitalWrite(PUTT_IN2, HIGH);
        digitalWrite(PUTT_IN3, LOW);
        digitalWrite(PUTT_IN4, LOW);
        break;
    }
  }
  else if(putt_select <= JOY_AVG) // Right putter
  {
    // move motor
    switch(motor_move_dir)
    {
      case CCW:
        digitalWrite(PUTT_IN1, HIGH);
        digitalWrite(PUTT_IN2, HIGH);
        digitalWrite(PUTT_IN3, HIGH);
        digitalWrite(PUTT_IN4, LOW);
        break;
      case CW:
        digitalWrite(PUTT_IN1, HIGH);
        digitalWrite(PUTT_IN2, HIGH);
        digitalWrite(PUTT_IN3, LOW);
        digitalWrite(PUTT_IN4, HIGH);
        break;
      case NO_DIR:
        digitalWrite(PUTT_IN1, HIGH);
        digitalWrite(PUTT_IN2, HIGH);
        digitalWrite(PUTT_IN3, HIGH);
        digitalWrite(PUTT_IN4, HIGH);
        break;
    }
  }
}

void drive_mode()
{
  Serial.println("DRIVE");
  drive();
  lifter();
}

void putter_mode()
{
  Serial.println("PUTT");
  putt();
}

void mode_manager()
{
  // read input from receiver
  int mode_input = pulseIn(MODE_SWITCH, HIGH);

  // determine mode from input
  if(mode_input < JOY_AVG)
  {
    // drive mode
    while(mode_input < JOY_AVG)
    {
      // read mode input from receiver
      mode_input = pulseIn(MODE_SWITCH, HIGH);

      // run drive code
      drive_mode();
    }
  }
  else if(mode_input > JOY_AVG)
  {
    // initialize the encoders
    // init_left_enc_pos = leftENC.read();
    // init_right_enc_pos = rightENC.read();
    
    // putt mode
    while(mode_input > ((JOY_MIN + JOY_MAX)/2))
    {
      // read mode input from receiver
      mode_input = pulseIn(MODE_SWITCH, HIGH);

      // run putt code
      putter_mode();
    }
  }
}
