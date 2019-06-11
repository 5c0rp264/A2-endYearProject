#include <Grove_I2C_Motor_Driver.h>

// default I2C address is 0x0f
#define I2C_ADDRESS 0x0f

const byte lineFollowSensorGG = 7;
const byte lineFollowSensorG = 8;
const byte lineFollowSensorD = 4;
const byte lineFollowSensorDD = 3;

int LFSensorValues[4]={0, 0, 0, 0};

short error = 0;
int integratedError = 0;
short previousError = 0;

#define Kp 25
#define Ki 1.3
short queueForIntegrate[3] = {0,0,0};
#define Kd 3


const byte interruptPin = 6;
int holeCounter = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Interruption are not fucking workin
 */



void setup(){
  pinMode(lineFollowSensorGG, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorG, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorD, INPUT); // initialize the digital pin as an output:
  pinMode(lineFollowSensorDD, INPUT); // initialize the digital pin as an output:
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, RISING );
  Serial.begin(9600);  // initialize serial communications at 9600 bps:
  Motor.begin(I2C_ADDRESS);

}


// the loop() method runs over and over again,
// as long as the Arduino has power
void loop(){
  //delay(100);
  Serial.println(holeCounter);
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
    }*/
    /*Serial.print("PID:");
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
    //This is an intersection
    //if (pathFindingSay = only a virage)
      /*getErrorCorrespondingToSensorValues();
      manageQueueWithNewError();
      int PIDValue = (Kp*error) + (Ki*integratedError) + (Kd*(error-previousError));
      /*if (PIDValue > 100 ){
        PIDValue = 100;
      }*/
      /*Serial.print("PID:");
      Serial.println(PIDValue);
      Serial.print("P:");
      Serial.println(error);
      Serial.print("I:");
      Serial.println(integratedError);
      Serial.print("D:");
      Serial.println((error-previousError));
      Motor.speed(MOTOR1,60+PIDValue);
      Motor.speed(MOTOR2,60-PIDValue);*/
    //If (pathFindingSay = realIntersect) {
    //act
    //}
    //Motor.stop(MOTOR1);
    //Motor.stop(MOTOR2);
    //exit(0);
  }
  
  //delay(100);


  previousError = error;
//  if(HIGH == digitalRead(signalPin))
//    Serial.println("black");
//  else  Serial.println("white");  // display the color
//  delay(1000);                  // wait for a second

}




void blink() {
  Serial.println("Detecting something");
  holeCounter++;
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
