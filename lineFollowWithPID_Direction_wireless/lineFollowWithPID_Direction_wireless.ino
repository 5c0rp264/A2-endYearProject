#include <VirtualWire.h>

#include <pathToTake.h>

#include <Grove_I2C_Motor_Driver.h>



// default I2C address is 0x0f
#define I2C_ADDRESS 0x0f

const byte lineFollowSensorGG = 7;
const byte lineFollowSensorG = 8;
const byte lineFollowSensorD = 4;
const byte lineFollowSensorDD = 5;

int LFSensorValues[4]={0, 0, 0, 0};

int8_t error = 0;
short integratedError = 0;
int8_t previousError = 0;

#define Kp 25
#define Ki 1.3
int8_t queueForIntegrate[3] = {0,0,0};
#define Kd 3


const byte interruptPin = 2;
short counter = 0;
int oldMillis = 0;
int timeForATurn = 0;

int8_t intersectionCounter = 0;

//bool waitForG = false, waitForD=false;


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Interruption are workin
 */

void setup(){
  pinMode(lineFollowSensorGG, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorG, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorD, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorDD, INPUT); // initialize the digital pin as an output:
  pinMode(interruptPin, INPUT_PULLUP);
  digitalWrite(interruptPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING);
  Serial.begin(9600);  // initialize serial communications at 9600 bps:
  Motor.begin(I2C_ADDRESS);
  oldMillis = millis();
}


// the loop() method runs over and over again,
// as long as the Arduino has power
void loop(){
  getCurrentSensorValues();
  int numOfSensorActivated = 0;
  for ( int i = 0; i<4; i++){
    if (LFSensorValues[i] == 1) {
      numOfSensorActivated++;
    }
  }
  if ( numOfSensorActivated < 2){
    getErrorCorrespondingToSensorValues();
    manageQueueWithNewError();
    int PIDValue = (Kp*error) + (Ki*integratedError) + (Kd*(error-previousError));
    /*if (PIDValue > 100 ){
      PIDValue = 100;
    }
    Serial.print("PID:");
    Serial.println(PIDValue);
    Serial.print("P:");
    Serial.println(error);
    Serial.print("I:");
    Serial.println(integratedError);
    Serial.print("D:");
    Serial.println((error-previousError));*/
    Motor.speed(MOTOR1,60+PIDValue);
    Motor.speed(MOTOR2,60-PIDValue);
  }else{
    if (((sizeof(pathToTake)/ sizeof(pathToTake[0]))+1)<= intersectionCounter){
      Motor.speed(MOTOR1,-100);
      Motor.speed(MOTOR2,-100);
      delay(40);
      Motor.stop(MOTOR1);
      Motor.stop(MOTOR2);
      exit(0);
    } else if (pathToTake[intersectionCounter] == 5){
      Motor.speed(MOTOR1,70);
      Motor.speed(MOTOR2,70);
      delay(175);
    } else if (pathToTake[intersectionCounter] == 1){
      Motor.speed(MOTOR1,-80);
      Motor.speed(MOTOR2,100);
      delay(150);
      while(digitalRead(lineFollowSensorD) == 0){}
    }else if (pathToTake[intersectionCounter] == 3){
      Motor.speed(MOTOR1,100);
      Motor.speed(MOTOR2,-80);
      delay(150);
      while(digitalRead(lineFollowSensorG) == 0){}
    }
    intersectionCounter++;

  }


  previousError = error;
}




void blink() {
  counter++;
  if (counter >= 20 ) {
    timeForATurn = millis()-oldMillis;
    oldMillis = millis();
    Serial.println(timeForATurn);
    counter = 0;
  }
}


void getCurrentSensorValues() {
  LFSensorValues[0] = digitalRead(lineFollowSensorGG);
  LFSensorValues[1] = digitalRead(lineFollowSensorG);
  LFSensorValues[2] = digitalRead(lineFollowSensorD);
  LFSensorValues[3] = digitalRead(lineFollowSensorDD);
}



void getErrorCorrespondingToSensorValues() {
  if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 1 )) error = 3;

  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 1 )&&(LFSensorValues[3]== 1 )) error = 2;

  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 1 )&&(LFSensorValues[3]== 0 )) error = 1;
  
  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 1 )&&(LFSensorValues[2]== 1 )&&(LFSensorValues[3]== 0 )) error = 0;

  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 1 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -1;

  else if((LFSensorValues[0]== 1 )&&(LFSensorValues[1]== 1 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -2;

  else if((LFSensorValues[0]== 1 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -3;
}


void manageQueueWithNewError(){ //this function is currently not working...
  integratedError = 0;
  for (int i=0; i<3; i++) {
    integratedError += queueForIntegrate[i];
    //Serial.print("Element en queue:");
    //Serial.println(queueForIntegrate[i]);
  } 
  for (int i=1; i<3; i++) {
    queueForIntegrate[i-1] = queueForIntegrate[i];
  }
  queueForIntegrate[2] = error;
}
