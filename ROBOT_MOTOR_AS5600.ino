#include <Wire.h>
#include <AS5600.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

AMS_5600 ams5600;

int ang, lang = 0;

#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);

int sine;
int many;
int sense;


#define LED 4
#define E_S 6
#define POT_W A0
#define POT A2
#define INA 7
#define INB 5
#define throt 9

int EN = 8;

float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  ang = retVal;
  return ang;
}


//this is to detect falling edge of the push button

int state, prevstate = 1;
int throttle = 120;
int ENA = 0;
int count;

void FALL_DETECT(){
  if(count == 1){
    state = digitalRead(6);
    count = 0;
    if(state == 0){
      if(prevstate == 1){
        ENA = !ENA;
        digitalWrite(LED, ENA);
      }
    }
    prevstate = state;
  }
  count++;
}

//------------------------------------------------
//-Motor control with hardware connection---------

void motorGo(int EN, int duty){
  if(EN){   
    if(duty > 0){
      digitalWrite(INA, HIGH);
      digitalWrite(INB, LOW);
      if(duty < 255){
        throttle = duty;
      }
      else{
        throttle = 255;
      }
    }
    else{
      digitalWrite(INA, LOW);
      digitalWrite(INB, HIGH);
      if(duty > -255){
        throttle = -duty;
      }
      else{
        throttle = 255;
      }
    }
    analogWrite(throt, throttle);
  }
  else{
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
    analogWrite(throt, 0);
  }
}

//----------------------------------------------------------------

//P controller: firstly I put and extremly high P (10 factor (the system was constantly oscillating) than Max forced me into wisdom and I tried P = 0.1 (there us offset but at least 
//the system can be consider compos mentis)
//P controL, on which I added a D term to add some damping. D term is easy (and honestly, I am wondering is it's really usefull)
// I then add the wonderful idea to add I term. The integral controller is a very naughty boy who turned nut the servo control.


double request;
double K = 0.1;
unsigned long previousTime, currentTime;
double elapsedTime;
double delta, lastError;
double KD = 0.05;
double KI = 0; // O, I was tired of seeing my system go crazy 
double rateError;
double cumError;
void P_CONT(double deg, double targ){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (double) currentTime - previousTime;
  lastError = delta;
  delta = targ-deg;
  rateError = (delta - lastError)/elapsedTime;   
  cumError += delta * elapsedTime;
  request = K*delta + KD*rateError + cumError*KI;
  throttle = (int) request;
  motorGo(ENA, throttle);  //call the motor function
}


//--------------------------------------------------------------------------------

//We add an angular feedback in the shape if a SAW. It was a pain in the ass, to use this angular mesurement in our controller (clogged non stop), So I decided to
//turn this periodic saw signal into an endless lign through (x, y) plane.
//double ang;
double ang_prev;
double ang_now;
double delta_ang;
double TOTAL;

double turnNum;
double inter;

void SUM(double s1, double s2){
  double diff = s2-s1;
  double inter;
  if(abs(diff)<250){
    TOTAL += diff;
  }
  else{
    if(diff < 0){
      inter = 360-s1+s2;
    }
    else{
      inter = -(360-s2+s1);
    }
    TOTAL += inter;
  }
}

//---------------------------------------------------------------------------------------------

double Target;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

pinMode(LED, OUTPUT);
pinMode(INA, OUTPUT);
pinMode(INB, OUTPUT);
pinMode(EN, OUTPUT);
pinMode(throt, OUTPUT);
pinMode(E_S, INPUT_PULLUP);
digitalWrite(EN, HIGH);
digitalWrite(LED, LOW);


//IC2 encoder librairy setup----------------------------------------------------------------------------------

Wire.begin();
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            SERIAL.print("Current Magnitude: ");
            SERIAL.println(ams5600.getMagnitude());
            break;
        }
        else{
            SERIAL.println("Can not detect magnet");
        }
        delay(1000);
    }
  }

//------------------------------------------------------------------------------------------------
//CAN BUS SETUP

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");

  //---------------------------------------------
}

void loop() {
  // put your main code here, to run repeatedly:



 //---This function retreives the DATA send by the can BUS, SPEED order, SENSE order.
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    many = canMsg.data[0];
    sine = canMsg.data[1]; 
  }

  if(sine == 2){
    sense = -1;
  }
  else if(sine == 1){
    sense = 1;
  }
 //-------------------------------------------------------------------------------
 //----
 FALL_DETECT(); //SAFETY DETECT
 
 convertRawAngleToDegrees(ams5600.getRawAngle());
 ang_prev = ang_now;
 ang_now = ang;
 SUM(ang_prev, ang_now);

 P_CONT(TOTAL, Target);
 Target += sense*many;  //creates a ramp input (higher many is
 /*Serial.print("   angle    :");
 Serial.print(ang);
 Serial.print("   Targ    :");
 Serial.print(Target);
 Serial.print("   angle diff   :");
 Serial.println(TOTAL);*/
 Serial.print("  sine  ");
 Serial.print(sine);
 Serial.print("   Targ    :");
 Serial.print(Target);
 Serial.print(" many ");
 Serial.println(many);
 delay(50);
}
