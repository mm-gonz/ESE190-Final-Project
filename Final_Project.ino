
#include "config.h"
#include <Servo.h>
#include <StopWatch.h>

// code sources: Labs 2 and 4
// https://docs.arduino.cc/learn/electronics/servo-motors
// https://playground.arduino.cc/Code/StopWatchClass/
// https://learn.adafruit.com/adafruit-io-basics-servo/arduino-code

// set up the feed
AdafruitIO_Feed *treats = io.feed("treats");
AdafruitIO_Feed *delayTime = io.feed("delayTime");

// create servo object to control motor
Servo myservo;

// variable to store servo position
int pos = 0;

// set up stop watch
StopWatch sw;
int treatDelay = 120000; // default delay time

// pins on the Feather board
#define trigPin 12
#define echoPin 13
const int redPin = 0;
const int greenPin = 15;

// values to send to the cloud
int lastTreatTime;
int treatCount = 1;

void setup() {
  // declare pins as inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  // attach the servo to the pin
  myservo.attach(16);
  myservo.write(pos);

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(!Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the 'delayTime' feed
  delayTime->onMessage(handleMessage);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  // begin stopwatch
  if(!sw.isRunning()) {
    sw.start();
    lastTreatTime = 0;
  }
  Serial.println("Stopwatch has started.");
  Serial.println(sw.elapsed());

  // send default delay time to adafruit in seconds
  delayTime->save(treatDelay/1000);
  
}

void loop() {
  // keep the client connected to io.adafruit.com
  io.run();

  long duration , distance;
  digitalWrite (trigPin , LOW ); // start trig at 0
  delayMicroseconds (2);
  digitalWrite (trigPin , HIGH ); // the rising edge of trig pulse
  delayMicroseconds (10); // decides duration of trig pulse
  digitalWrite (trigPin , LOW ); // falling edge of the trig pulse
  // NOTE: echo pin reads HIGH till it receives the reflected signal
  duration = pulseIn (echoPin , HIGH ); // reading the duration for which echoPin was HIGH gives
  // the time the sensor receives a reflected signal at the echo pin
  distance = (duration / 2) / 29.1; // calculate the distance of the reflecting surface in cm

  Serial.println("Distance is:    ");
  Serial.println(distance);
  
  // this section dispenses treats based on distance
  if (distance<50 && distance>=0.1) { 
    // make sure treats are not constantly dispensed
    int elapsedTime = sw.elapsed() - lastTreatTime;
    Serial.println(lastTreatTime);
    Serial.println(elapsedTime);
    if (elapsedTime >= treatDelay || lastTreatTime == 0) { // check for the right delay, or if it just started running
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, LOW);
      Serial.println(sw.elapsed());
      lastTreatTime = sw.elapsed(); // get the timestamp for when the treat is dispensed
      treats->save(treatCount);
      delay(2000);

      // move the motor
      for (pos = 0; pos < 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
      for (pos = 180; pos > 0; pos -= 1) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }

        
    Serial.println("Treat dispensed!");
  } else {
      digitalWrite(greenPin, LOW);
      digitalWrite(redPin, HIGH);
      Serial.println("It's only been ");
      Serial.println(elapsedTime / 60000);
      Serial.println("minutes");
      Serial.println("Delay time is ");
      Serial.println(treatDelay / 60000);
      treats->save(0);
    }
  }

  // Only check every 3 seconds so that the adafruit feed is not overloaded
  delay(3000);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, LOW);
}

// this function is called whenever a 'delayTime' message is received from Adafruit IO
void handleMessage(AdafruitIO_Data *data) {
  // convert the data to integer
  int newDelay = data->toInt();
  treatDelay = newDelay*1000;

  Serial.println("New delay time is");
  Serial.println(newDelay);
  Serial.println("seconds");
}
