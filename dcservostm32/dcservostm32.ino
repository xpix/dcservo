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

//Motor PWM output
#define X_PWM         PB8 //hardware timer 

//Motor STEP inputs
#define X_STEP        PB4

//Motor DIR inputs
#define X_DIR         PB15

//Sensorless Homing/motor enable
#define ENABLE        PA11

/******************PID/Encoder stuff*****************/
//array of kP, kI, kD for all three axes, X, Y, Z
double pid[3][3] = { {50,10,0.06},{50,10,0.06},{50,10,0.06} } ;

//destination locations
long target_x=0;
  
double x_input=0, x_output=0, x_setpoint=0;

PID xPID(&x_input, &x_output, &x_setpoint, pid[0][0],pid[0][1],pid[0][2], DIRECT);

//long ints[5] = {0, 0, 0, 0, 0};

//Hardware timer encoders
quadCounter X_ENCODER(QUAD_TIMER_3);

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


void setup() { 

  //Outputs
  pinMode(X_M1, OUTPUT); 
  pinMode(X_M2, OUTPUT);
  pinMode(PC13, OUTPUT); //LED
  
  //PWM
  SET_PWM(X_PWM);

  //Hardware timer setup for motor driver PWM
  timer[2] = new HardwareTimer(2);
  timer[2]->pause();
  timer[2]->setPeriod(50); //20 kHz
  timer[2]->resume();
  
  //Interrupts and Inputs
  pinMode(X_STEP, INPUT);
  attachInterrupt(X_STEP, countStep_X, RISING);  
  pinMode(X_DIR, INPUT);
  pinMode(ENABLE, INPUT);

  //Serial
  Serial2.begin(115200);
  
  //Setup the pids 
  xPID.SetMode(AUTOMATIC);
  xPID.SetSampleTime(1);
  xPID.SetOutputLimits(-65535,65535);
  
  //retrieveSettings();
  update_PIDs();

  //reset counters to 0
  X_ENCODER.placeCount(0);

  WRITE(PC13, HIGH);
} 

void loop(){
   
    process_commands();
    
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

}

void process_commands() {
  char cmd = Serial2.read();
  static float p,i,d;
  int pos;
  
  if(cmd>'Z') cmd-=32;
  switch(cmd) {
     case 'X': current_axis = 0; break;
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
}

void setencoder_pos(int pos) {
  if (current_axis == 0)  X_ENCODER.placeCount(pos);
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
}

void report_positions() {
  Serial2.print("Enabled: ");
  Serial2.print(READ(ENABLE));
  Serial2.print(" X encoder: ");
  Serial2.print(encoder_counts((int)X_ENCODER.count(), 3));
  Serial2.print(" Target X: ");
  Serial2.print(target_x);
  Serial2.println(" END");
}

void sendPID() {
  Serial2.print("X PID: ");
  for (int x=0; x<3; x++) {
      Serial2.print(pid[0][x]);
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

