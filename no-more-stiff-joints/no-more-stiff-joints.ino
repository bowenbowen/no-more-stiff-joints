#include <Wire.h>


#define buzzerPin 5
#define xPin A0
#define yPin A3
#define zPin A5

String inString = "";
int countDown = -1;
int alertTime = 10;   // how often you get a alert
int ifActivated = false;      // whether the who system is activated 


int valueX;
int valueY;
int valueZ;

int lastValueX, lastValueY, lastValueZ;      // the last angle difference
int moveX, moveY, moveZ, moveAverage;        // how much has changed since the last state

const int sessionLength = 5;    // monitor how many values back to see if currently active
int moveTolerance = 50;    // how much move is deemed active
int lastMoveVals[sessionLength];
int activeLimit = 2;     // out of [sessionLength] number of values, if over this value, it is currently active
int isActive = false;      // whether active or not in the current session 



/*********************************************************************bl
  Bluefruit LE Connect Plotter 
  for Feather Bluefruit -> Bluefruit LE Connect app
  
  outputs dummy values for demo use with BLuefruit LE Connect app
  change SEND_SECOND_PLOT define to 1 to output second plot using sine wave table

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "sine.h"

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define SEND_SECOND_PLOT            0
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  pinMode(buzzerPin, OUTPUT);

  
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}




void loop(void) {
  delay(1000);                  //wait 1 second

  valueX = analogRead(xPin);
  valueY = analogRead(yPin);
  valueZ = analogRead(zPin);

  // how much has changed since the last state
  moveX = valueX - lastValueX;
  moveY = valueY - lastValueY;
  moveZ = valueZ - lastValueZ;
  moveAverage = (abs(moveX) + abs(moveY) + abs(moveZ)) / 3;


  // update last values
  for (byte i = sessionLength - 1; i > 0; i -= 1){ lastMoveVals[i] = lastMoveVals[i-1]; }
  lastMoveVals[0] = moveAverage;


  // check if the current session is active. 
  // if more than [activeLimit] number of values are over [moveTolerance], it is active
  int activeCount = 0;
  for (byte i = 0; i < sessionLength -1; i += 1){
    if (abs(lastMoveVals[i]) > moveTolerance){ activeCount += 1; }
  }
  if (activeCount >= activeLimit){
    isActive = true;
  } else {
    isActive = false; 
  }


  // prepare to read the next state
  lastValueX = valueX;
  lastValueY = valueY;
  lastValueZ = valueZ;

//  uint8_t val_x = map(moveX, -150, 150, 0, 255);
//  uint8_t val_y = map(moveY, -150, 150, 0, 255);
//  uint8_t val_z = map(moveZ, -150, 150, 0, 255);
  uint8_t val_ave = map(moveAverage, 0, 150, 0, 255);

  
//  ble.print(val_x);               //send value to bluefruit uart
//  ble.print(",");
//  ble.print(val_y);
//  ble.print(",");
//  ble.print(val_z);

//  ble.print(moveX);               //send value to bluefruit uart
//  ble.print(",");
//  ble.print(moveY);
//  ble.print(",");
//  ble.print(moveZ);
//  ble.print(moveAverage);

  ble.print(lastMoveVals[0]);
  ble.print(",");
  ble.print(lastMoveVals[1]);
  ble.print(",");
  ble.print(lastMoveVals[2]);
  ble.print(",");
  ble.print(lastMoveVals[3]);
  ble.print(",");
  ble.print(lastMoveVals[4]);
  ble.print(", ifActive:");
  ble.print(isActive);
  

  if (SEND_SECOND_PLOT) {               //change SEND_SECOND_PLOT to 1 for add'l sine plot
    if (sineIndex > 255) sineIndex = 0; //stay within bounds of sine table
    ble.print(",");                     //print delimiter for second plot
    ble.print(sine_wave[sineIndex]);    //print value from sine table
    sineIndex++;                        //increment index
  }

  ble.println();  //print newline so app knows to plot the values

  while (ble.available()){
    int inChar = ble.read();
    if(isDigit(inChar)){
      inString += (char)inChar;
    }
    if(inChar == '\n'){
      ifActivated = true;
      
      Serial.print("Value: ");
      Serial.println(inString.toInt());
      countDown = inString.toInt();
      alertTime = inString.toInt();
      Serial.print("String: ");
      Serial.println(inString);
      // clear the string for new input:
      inString = "";
    }
  }
 
  
//  switch(countDown){
//    case 0:
//      digitalWrite(buzzerPin, LOW);
//      break;
//    case 1:
//      digitalWrite(buzzerPin, HIGH);
//      break;
//    case 10:
//      digitalWrite(buzzerPin, HIGH);
//      break;
//  }

  // reset countdown if the current session is active. Otherwise, keep counting down
  if (ifActivated == true && isActive == true){
    countDown = alertTime;
  }
  
  if(countDown == 0){
    Serial.print("Time's up! ");
    Serial.println(countDown);
    digitalWrite(buzzerPin, HIGH);
    countDown = 0;
  } else if (countDown > 0){
    countDown = countDown - 1;
    Serial.print("Time left: ");
    Serial.println(countDown);
    digitalWrite(buzzerPin, LOW);
  }

}
