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

#define Kp 20
#define Ki 1.5
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
  getCurrentSensorValues(); //explicit
  int numOfSensorActivated = 0;
  for ( int i = 0; i<4; i++){ //count number of sensor activated
    if (LFSensorValues[i] == 1) {
      numOfSensorActivated++;
    }
  }
  if ( numOfSensorActivated < 2){ //if is not an intersection -> follow line
    getErrorCorrespondingToSensorValues(); //explicit
    manageQueueWithNewError(); //integrate with new error
    int PIDValue = (Kp*error) + (Ki*integratedError) + (Kd*(error-previousError));
    /*if (PIDValue > 100 ){
      PIDValue = 100;
    }*/
    /*Serial.print("PID:");
    Serial.println(PIDValue);
    Serial.print("P:");
    Serial.println(error*Kp);
    Serial.print("I:");
    Serial.println(integratedError*Ki);
    Serial.print("D:");
    Serial.println((error-previousError*Kd));*/
    Motor.speed(MOTOR1,60+PIDValue);//assign adequate value with new PID 
    Motor.speed(MOTOR2,60-PIDValue);//assign opposit value for oppsit motor
  }else{
    if (((sizeof(pathToTake)/ sizeof(pathToTake[0])))< intersectionCounter){//if we have done all order -> stop
      Motor.speed(MOTOR1,-100);
      Motor.speed(MOTOR2,-100);
      delay(40);
      Motor.stop(MOTOR1);
      Motor.stop(MOTOR2);
      exit(0);
    } else if (pathToTake[intersectionCounter] == 5){ //order go forward
      Motor.speed(MOTOR1,70);
      Motor.speed(MOTOR2,70);
      delay(175);
    } else if (pathToTake[intersectionCounter] == 1){//order turn right
      Motor.speed(MOTOR1,-55);
      Motor.speed(MOTOR2,100);
      delay(150);
      while(digitalRead(lineFollowSensorD) == 0){}
    }else if (pathToTake[intersectionCounter] == 3){//order turn left
      Motor.speed(MOTOR1,100);
      Motor.speed(MOTOR2,-55);
      delay(150);
      while(digitalRead(lineFollowSensorG) == 0){}
    }else if (pathToTake[intersectionCounter] == 2){//order turn back
      Motor.speed(MOTOR1,90);
      Motor.speed(MOTOR2,-90);
      delay(1300);
    }
    intersectionCounter++;

  }


  previousError = error;
}




void blink() { //count each hole to get speed of the robot
  counter++;
  if (counter >= 20 ) {
    timeForATurn = millis()-oldMillis;
    oldMillis = millis();
    Serial.println(timeForATurn);
    counter = 0;
  }
}


void getCurrentSensorValues() { //explicit
  LFSensorValues[0] = digitalRead(lineFollowSensorGG);
  LFSensorValues[1] = digitalRead(lineFollowSensorG);
  LFSensorValues[2] = digitalRead(lineFollowSensorD);
  LFSensorValues[3] = digitalRead(lineFollowSensorDD);
}



void getErrorCorrespondingToSensorValues() { //explicit
  if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 1 )) error = 3;

  //else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 1 )&&(LFSensorValues[3]== 1 )) error = 2;

  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 1 )&&(LFSensorValues[3]== 0 )) error = 2;
  
  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = 0;

  else if((LFSensorValues[0]== 0 )&&(LFSensorValues[1]== 1 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -2;

  //else if((LFSensorValues[0]== 1 )&&(LFSensorValues[1]== 1 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -2;

  else if((LFSensorValues[0]== 1 )&&(LFSensorValues[1]== 0 )&&(LFSensorValues[2]== 0 )&&(LFSensorValues[3]== 0 )) error = -3;
}


void manageQueueWithNewError(){ //manageQueue for later on integrate the last error values
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
