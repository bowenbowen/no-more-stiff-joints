/***************************** HAVE DONE *****************************/
// See how much the sensor is moving by comparing the current readings with the last sets
// The larger a move, the larger the move values are; if still, the move values stay close to 0


/***************************** TO DO *****************************/
// 1. Monitor the activity within a period of time. If deemed inactive, send vibration alert
// 2. Use BLE to change the length of the session



//current readings
int valueX;
int valueY;
int valueZ;

int lastValueX, lastValueY, lastValueZ;  // the last angle difference
int moveX, moveY, moveZ;        // how much has changed since the last state

int moveTolerance = 100;   // how much move is deemed active
int sessionInMins = 2;       // within how many minutes, continuous inactivity will trigger to a reminder
unsigned long sessionLength = sessionInMins * 60000;  // the value above in millis
int isActive = false;      // whether active or not in the current session 
int vibration = false;     // vibration ON/OFF


void setup(){
  Serial.begin(9600);      // sets the serial port to 9600
}


void loop(){
  
  valueX = analogRead(A0);
  valueY = analogRead(A1);
  valueZ = analogRead(A2);


  // how much has changed since the last state
  moveX = valueX - lastValueX;
  moveY = valueY - lastValueY;
  moveZ = valueZ - lastValueZ;

  // prepare to read the next state
  lastValueX = valueX;
  lastValueY = valueY;
  lastValueZ = valueZ;
  
//  Serial.print("X:");
//  Serial.print(valueX);
//  Serial.print("  Y:");
//  Serial.print(valueY);
//  Serial.print("  Z:");
//  Serial.println(valueZ);

  
  Serial.print("X move:");
  Serial.print(moveX);
  Serial.print("  Y move:");
  Serial.print(moveY);
  Serial.print("  Z move:");
  Serial.println(moveZ);
  
  delay(1000);

}
