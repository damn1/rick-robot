/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

// ros library
#include <ros.h>
// messaggi contenenti RPM data
#include <std_msgs/Float64.h>
// libreria per la gestione dei 2 motori
#include "DualVNH5019MotorShield.h"
// easy encoder
#include <Encoder.h>

// frequenza di pubblicazione costante in millis
#define FREQ_CTRL 10
// variabile contenitore di ciò che il controller pubblica
float left_ctrl_effort_storage;
float right_ctrl_effort_storage;

/**
 * Stoppa il motore in caso di problemi, entra in loop infinito
 */
void stopIfFault();
/**
 * CallBack per gestire il messaggio in arrivo dal PID controller
 */
void left_ctrl_eff_call_back( const std_msgs::Float64& ctrl_effort_msg);
void right_ctrl_eff_call_back( const std_msgs::Float64& ctrl_effort_msg);

ros::NodeHandle node_handle;
/**
 * message per contenere RPM misurati dal sensore encoder.
 */
std_msgs::Float64 left_rpm_out;
std_msgs::Float64 right_rpm_out;
/**
 * Publisher per pubblicare RPM data message
 * Pubblica dati per il controller del pid.
 */
ros::Publisher left_plant_state("/left_wheel/rpm_plant_state_from_arduino", &left_rpm_out);
ros::Publisher right_plant_state("/right_wheel/rpm_plant_state_from_arduino", &right_rpm_out);
/**
 * Subscriber per ricevere cmd data da passare all'attuatore [Volt].
 * Riceve dato dal controller PID.
 */
ros::Subscriber<std_msgs::Float64> left_control_effort("/left_wheel/ctrl_effort_to_arduino", &left_ctrl_eff_call_back);
ros::Subscriber<std_msgs::Float64> right_control_effort("/right_wheel/ctrl_effort_to_arduino", &right_ctrl_eff_call_back);

DualVNH5019MotorShield motorShield;
Encoder encoderRight(2,13);
Encoder encoderLeft(3,11);

// dati per gestire gli encoder
long newPositionRight;
long oldPositionRight = -999;

long newPositionLeft;
long oldPositionLeft = -999;

long lastmillis = 0;

void setup()
{ 
  lastmillis = millis();
  pinMode(13, OUTPUT);
  //Serial.begin(57600);
  node_handle.initNode();
  // per il publisher
  node_handle.advertise(left_plant_state);
  node_handle.advertise(right_plant_state);
  // per il subscriber
  node_handle.subscribe(left_control_effort);
  node_handle.subscribe(right_control_effort);
  motorShield.init();
}

void loop()
{  
/*
  if ((millis() - lastmillis) > FREQ_CTRL) {
    left_plant_state.publish(&left_rpm_out);
    lastmillis = millis();
  }
  */
  node_handle.spinOnce();
  
  newPositionLeft  = encoderLeft.read();
  float rpmLeft = getmotordata((millis() - lastmillis), (newPositionLeft - oldPositionLeft));
  
  newPositionRight  = encoderRight.read();
  float rpmRight = getmotordata((millis() - lastmillis), (newPositionRight - oldPositionRight));
  
  lastmillis = millis();
  if (newPositionLeft != oldPositionLeft) oldPositionLeft = newPositionLeft;
  if (newPositionRight != oldPositionRight) oldPositionRight = newPositionRight;

  left_rpm_out.data = rpmLeft;
  right_rpm_out.data = rpmRight;
  
  left_plant_state.publish(&left_rpm_out);
  right_plant_state.publish(&right_rpm_out);

  delay(100);
}


void stopIfFault()
{
  if (motorShield.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (motorShield.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}


void left_ctrl_eff_call_back( const std_msgs::Float64& ctrl_effort_msg){
  left_ctrl_effort_storage = ctrl_effort_msg.data;
  motorShield.setM2Speed(ctrl_effort_msg.data);
}

void right_ctrl_eff_call_back( const std_msgs::Float64& ctrl_effort_msg){
  right_ctrl_effort_storage = ctrl_effort_msg.data;
  motorShield.setM1Speed(ctrl_effort_msg.data);
}

float getmotordata(long deltatime, long deltacount) {
  return (deltacount * 60 * 1000 )/(deltatime * 64 * 70);
}

