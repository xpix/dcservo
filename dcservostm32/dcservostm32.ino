/*
 * dcservo for STM32 "BluePill"
 * Based on original dcservo from Miguel Sanchez: https://github.com/misan/dcservo
 * Uses hardware timers to read quadrature encoders, using the library from: https://github.com/chrisalbertson/quadratureBluePill
 * Using fastio macros from Marlin Firmware: https://github.com/MarlinFirmware/Marlin
 * Uses Roger Clarke's STM32duino core
 * Uses PID for arduino library: 
 * 
*/

#include <EEPROM.h>
#include "quadratureBluePill.h"
#include "fastio_STM32F1.h"
#include <PID_v1.h>

/*************Pin defintions***************/
//Motor direction output
#define X_M1          PA15
#define X_M2          PB3
#define Y_M1          PA5
#define Y_M2          PA4
#define Z_M1          PB1
#define Z_M2          PB0

//Motor PWM output
#define X_PWM         PB8 //hardware timer 
#define Y_PWM         PB9 //hardware timer
#define Z_PWM         PA0 //hardware timer 

//Motor STEP inputs
#define X_STEP        PB4
#define Y_STEP        PB5
#define Z_STEP        PA10

//Motor DIR inputs
#define X_DIR         PB15
#define Y_DIR         PB11
#define Z_DIR         PA1

#define ENDSTOP       PB12
#define HOMING        PB10

//Sensorless Homing/motor enable
#define ENABLE        PA11

/******************PID/Encoder stuff*****************/
//array of kP, kI, kD for all three axes, X, Y, Z
double pid[3][3] = { {50,10,0.06},{50,10,0.06},{50,10,0.06} } ;

//destination locations
long target_x=0, target_y=0, target_z=0;
  
double x_input=0, x_output=0, x_setpoint=0;
double y_input=0, y_output=0, y_setpoint=0;
double z_input=0, z_output=0, z_setpoint=0;

PID xPID(&x_input, &x_output, &x_setpoint, pid[0][0],pid[0][1],pid[0][2], DIRECT);
PID yPID(&y_input, &y_output, &y_setpoint, pid[1][0],pid[1][1],pid[1][2], DIRECT);
PID zPID(&z_input, &z_output, &z_setpoint, pid[2][0],pid[2][1],pid[2][2], DIRECT);

//long ints[5] = {0, 0, 0, 0, 0};

//Hardware timer encoders
quadCounter Z_ENCODER(QUAD_TIMER_1);
quadCounter X_ENCODER(QUAD_TIMER_3);
//quadCounter Y_ENCODER(QUAD_TIMER_4);

//Sadly, Y-axis is using software interupts because I missed up the pin assignments on the timers for PWM.
const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
static unsigned char New, Old;
volatile long encoderYPos = 0;
void encoderInt() { 
  Old = New;
  New = READ(PB7)*2 + READ(PB6);
  encoderYPos+= QEM [Old * 4 + New];
}

/************************Some globally defined things*********************/
//Current position, previous position, motor output, and number of hits (for homing) for each axis (X,Y,Z)
long positions[4][3] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} };
//endstop related stuff
boolean homed=false;
int HITLIMIT=20;
//global defintion of axis for setting PIDs, 0 = X, 1 = Y, 2 = Z
byte current_axis;

/**********************Step Counting Interrupts**************************/   
void countStep_X() {
   if READ(X_DIR) {
      target_x--;
   } else { target_x++;}
}

void countStep_Y() {
   if READ(Y_DIR) {
      target_y--;
   } else { target_y++; }
}

void countStep_Z() {
   if READ(Z_DIR) {
      target_z--;
   } else { target_z++; }
}


void setup() { 

  //Outputs
  pinMode(X_M1, OUTPUT); 
  pinMode(X_M2, OUTPUT);
  pinMode(Y_M1, OUTPUT); 
  pinMode(Y_M2, OUTPUT);
  pinMode(Z_M1, OUTPUT); 
  pinMode(Z_M2, OUTPUT);
  pinMode(PC13, OUTPUT); //LED
  pinMode(ENDSTOP, OUTPUT);
  
  //PWM
  SET_PWM(X_PWM);
  SET_PWM(Y_PWM);
  SET_PWM(Z_PWM);
  //Hardware timer setup for motor driver PWM
  timer[2] = new HardwareTimer(2);
  timer[2]->pause();
  timer[2]->setPeriod(50); //20 kHz
  timer[2]->resume();
  timer[4] = new HardwareTimer(4);
  timer[4]->pause();
  timer[4]->setPeriod(50);
  timer[4]->resume();  
  
  //Interrupts and Inputs
  pinMode(PB6, INPUT_PULLUP); //Y-axis
  pinMode(PB7, INPUT_PULLUP); //Y-axis
  pinMode(X_STEP, INPUT);
  pinMode(Y_STEP, INPUT);
  pinMode(Z_STEP, INPUT);
  attachInterrupt(X_STEP, countStep_X, RISING);  
  attachInterrupt(Y_STEP, countStep_Y, RISING);
  attachInterrupt(Z_STEP, countStep_Z, RISING);
  pinMode(X_DIR, INPUT);
  pinMode(Y_DIR, INPUT);
  pinMode(Z_DIR, INPUT);  
  pinMode(HOMING, INPUT);
  pinMode(ENABLE, INPUT);
  attachInterrupt(PB6, encoderInt, CHANGE);    //Y-axis interrupts
  attachInterrupt(PB7, encoderInt, CHANGE);    //Y-axis interrupts

  //Serial
  Serial2.begin(115200);
  
  //Setup the pids 
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(1);
  xPID.SetOutputLimits(-65535,65535);

  yPID.SetMode(AUTOMATIC);
  yPID.SetSampleTime(1);
  yPID.SetOutputLimits(-65535,65535);

  zPID.SetMode(AUTOMATIC);
  zPID.SetSampleTime(1);
  zPID.SetOutputLimits(-65535,65535);
  
  //retrieveSettings();
  update_PIDs();

  //HardwareTimer encoder modifications
  //Z axis, cutdown on number of encoder counts
  timer[1]->pause();
  timer[1]->setEdgeCounting(TIMER_SMCR_SMS_ENCODER1);
  timer[1]->resume();
  //X axis
  //timer[3]->pause();
  //timer[3]->attachInterrupt(0, func3);
  //timer[3]->resume();
  //reset counters to 0
  X_ENCODER.placeCount(0);
  Z_ENCODER.placeCount(0);

  WRITE(PC13, HIGH);
  digitalWrite(ENDSTOP, LOW); // pull low, high to trigger endstop
  digitalWrite(HOMING, LOW); // not in homing mode by default
} 

void loop(){
   
    process_commands();
    
    if (homed) { WRITE(ENDSTOP,LOW); WRITE(PC13, HIGH); delay(100); homed = false; }
    bool enabled = READ(ENABLE);
    bool homing = READ(HOMING);
    
    //process X-axis
    positions[0][0] = encoder_counts((int)X_ENCODER.count(), 3);
    x_input = positions[0][0];
    x_setpoint=target_x;
    while(!xPID.Compute());
     
    if (x_output > 0) {
      WRITE(X_M1, 1); WRITE(X_M2, 0);
    } else {
      WRITE(X_M1, 0); WRITE(X_M2, 1);
    }
    
    positions[2][0] = abs(x_output);
    
    if (!enabled) x_output=0;
    pwmWrite(X_PWM, abs(x_output));
    
    //process Y-axis
    y_input = encoderYPos;
    positions[0][1]= encoderYPos;
    y_setpoint=target_y;
    while(!yPID.Compute());

    positions[2][1] = abs(y_output);
    
    if (y_output > 0) {
      WRITE(Y_M1, 1); WRITE(Y_M2, 0);
    } else {
      WRITE(Y_M1, 0); WRITE(Y_M2, 1);
    }
    
    if (!enabled) y_output=0;
    pwmWrite(Y_PWM, abs(y_output));
     
    //process Z-axis
    positions[0][2] = encoder_counts((int)Z_ENCODER.count(), 1);
    z_input = positions[0][2];
    z_setpoint = target_z;
    while(!zPID.Compute());
    if (z_output > 0) {
      WRITE(Z_M1, 1); WRITE(Z_M2, 0);
    } else {
      WRITE(Z_M1, 0); WRITE(Z_M2, 1);
    }

    positions[2][2] = abs(z_output);
    
    if (!enabled)  z_output=0;
    pwmWrite(Z_PWM, abs(z_output));

    //Sensorless homing monitors positional change when homing flag is high and if the motor PID output is above 2.8K will count hits
    //All axes use the same pin as endstop and same homing signal, so we have to home indivdiually, and cycle through since we don't know which
    //is being homed. Only the output lets us know.
    //Requires Marlin firmware modification to output homing signal
    if (homing) {
      WRITE(PC13, LOW);
      //set the encoder "sensitivity", i.e. how different readings must be to indicate encoder stopped "hit"
      int sens=40;
      for (int x=0; x<3; x++) {
        //modify sensitivity for z-axis, there are 400 steps/mm, so set it to about 0.5 mm
        if (x == 2) sens=200;
        if (abs(positions[0][x] - positions[1][x]) < sens && positions[2][x] > 1000) positions[3][x] = positions[3][x] + 1;
        if (positions[3][x] > HITLIMIT) { WRITE(ENDSTOP, HIGH); stop_motors(); homed = true; delay(200); positions[3][x] = 0; }
      }
      
      for (int x=0; x<3; x++) { positions[1][x] = positions[0][x]; }
    }

}

void process_commands() {
  char cmd = Serial2.read();
  static float p,i,d;
  int pos;
  
  if(cmd>'Z') cmd-=32;
  switch(cmd) {
     case 'X': current_axis = 0; break;
     case 'Y': current_axis = 1; break;
     case 'Z': current_axis = 2; break;
     case 'P': p = Serial2.parseFloat(); update_P(p); break;
     case 'I': i = Serial2.parseFloat(); update_I(i); break;
     case 'D': d = Serial2.parseFloat(); update_D(d); break;
     case 'S': pos = Serial2.parseInt(); setencoder_pos(pos); break;
     case 'R': sendPID(); break;
     case 'Q': report_positions(); break;
 }

if (p || i || d) update_PIDs();

}

void stop_motors() {
   pwmWrite(X_PWM, 0);
   pwmWrite(Y_PWM, 0);
   pwmWrite(Z_PWM, 0);
}

void setencoder_pos(int pos) {
  if (current_axis == 0)  X_ENCODER.placeCount(pos);
  if (current_axis == 1)  encoderYPos = pos;
  if (current_axis == 2)  Z_ENCODER.placeCount(pos);
}

void update_P(float p) {
  pid[current_axis][0] = p;
}

void update_I(float i) {
  pid[current_axis][1] = i;
}

void update_D(float d) {
  pid[current_axis][2] = d;
  
}


//Gives absolute encoder position, to avoid dealing with timer overflow
int32_t encoder_counts(int encoder, int int_pos) {
  int32_t result = ints[int_pos]*PPR;
  result = result+encoder;
  return result;
}

void update_PIDs() {

    xPID.SetTunings(pid[0][0],pid[0][1],pid[0][2]);
    yPID.SetTunings(pid[1][0],pid[1][1],pid[1][2]);
    zPID.SetTunings(pid[2][0],pid[2][1],pid[2][2]);
}

void report_positions() {
  Serial2.print("Enabled: ");
  Serial2.print(READ(ENABLE));
  Serial2.print(" X encoder: ");
  Serial2.print(encoder_counts((int)X_ENCODER.count(), 3));
  Serial2.print(" Y encoder: ");
  Serial2.print(encoderYPos);
  //Serial2.print(encoder_counts((int)Y_ENCODER.count(), 4));
  Serial2.print(" Z encoder: ");
  Serial2.print(encoder_counts((int)Z_ENCODER.count(), 1));
  Serial2.print(" Target X: ");
  Serial2.print(target_x);
  Serial2.print(" X output: ");
  Serial2.print(x_output);
  Serial2.print(" Target Y: ");
  Serial2.print(target_y);
  Serial2.print(" Y output: ");
  Serial2.print(y_output);
  Serial2.print(" Target Z: ");
  Serial2.print(target_z);
  Serial2.print(" Z output: ");
  Serial2.print(z_output);
  Serial2.println(" END");
}

void sendPID() {
  Serial2.print("X PID: ");
  for (int x=0; x<3; x++) {
      Serial2.print(pid[0][x]);
      Serial2.print(" ");
  }
  Serial2.print("Y PID: ");
  for (int x=0; x<3; x++) {
      Serial2.print(pid[1][x]);
      Serial2.print(" ");
  }
  Serial2.print("Z PID: ");
  for (int x=0; x<3; x++) {
      Serial2.print(pid[2][x]);
      Serial2.print(" ");
  }
  Serial2.println("END"); 
}

void writeSettings() {
  //EEPROM.put(0,pid);
}

void retrieveSettings() {
  //EEPROM.get(0,pid);
}

