#include <ams_as5048b.h>

#define LED 4
#define E_S 6
#define POT_W A0
#define POT A2
#define INA 7
#define INB 5
#define throt 9
//#define EN 8
int EN = 8;
AMS_AS5048B mysensor;

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

double request;
double K = 0.1;
unsigned long previousTime, currentTime;
double elapsedTime;
double delta, lastError;
double KD = 0;
double rateError;
void P_CONT(double deg, double targ){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (double) currentTime - previousTime;
  lastError = delta;
  delta = targ-deg;
  rateError = (delta - lastError)/elapsedTime;
  request = K*delta + KD*rateError;
  throttle = (int) request;
  motorGo(ENA, throttle);
}

double ang;
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

while (!Serial) ; //wait until Serial ready

  //Start Wire object. Unneeded here as this is done (optionally) by the AMS_AS5048B object (see lib code - #define USE_WIREBEGIN_ENABLED)
  //Wire.begin();

  //init AMS_AS5048B object
  mysensor.begin();

  //consider the current position as zero
  mysensor.setZeroReg();
}

void loop() {
  // put your main code here, to run repeatedly:
 FALL_DETECT();
 
 mysensor.angleR(U_RAW, true);
 ang = mysensor.angleR(U_DEG, false);
 ang_prev = ang_now;
 ang_now = ang;
 SUM(ang_prev, ang_now);

 P_CONT(TOTAL, Target);
 Target += 10;
 Serial.print("   angle    :");
 Serial.print(ang);
 Serial.print("   Targ    :");
 Serial.print(Target);
 Serial.print("   angle diff   :");
 Serial.println(TOTAL);
 delay(50);
}
