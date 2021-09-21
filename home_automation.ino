#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

int livingroom = 5;;
int diningroom = 7;
Servo myServo;
int TxD = 11;
int RxD = 10;
int servoposition;
int servopos;
int new1;
SoftwareSerial Bluetooth(TxD, RxD);
int motorPin = 3;

char Data;
void sendData(String transmitData){
Bluetooth.println(transmitData);}

void setup(){
    Bluetooth.begin(9600);
    pinMode(livingroom,OUTPUT);
    pinMode(diningroom,OUTPUT);
    pinMode(motorPin,OUTPUT);
    int pos=0;
    myServo.attach(9);
    myServo.write(0);
    Serial.begin(9600);
}

void loop(){
    if(Bluetooth.available()){
        Data=Bluetooth.read();
       // String value = Bluetooth.readString();
      //  servoposition = value.toInt();
        if(Data==('4')){
            digitalWrite(livingroom,1);
            sendData("Living Room Light ON");
        }
        
        if(Data==('5')){
            digitalWrite(livingroom,0);
            sendData("Living Room Light OFF");
        }

        if (Data==('1')){
            Serial.println(servoposition);
            myServo.write(0);
      }
        
        if (Data == ('2')){
            Serial.println(servoposition);
            myServo.write(90);
      }

        if (Data == ('3')){
            Serial.println(servoposition);
            myServo.write(180);
      }
    
        if(Data == ('6')){
            digitalWrite(diningroom,1);
            sendData("Dining Room Light ON");
        }
        if(Data==('7')){
            digitalWrite(diningroom,0);
            sendData("Dining Room Light OFF");
        }
        if(Data==('a')){
            digitalWrite(livingroom,1);
         //   digitalWrite(bedroom,1);
            digitalWrite(diningroom,1);
            sendData("ALL LIGHTS ON");
        }
        if(Data==('b')){
            digitalWrite(livingroom,0);
        //    digitalWrite(bedroom,0);
            digitalWrite(diningroom,0);
            sendData("ALL LIGHTS OFF");
        }
        if (Data==('c')){
          digitalWrite(motorPin,1);
        }
        if (Data==('d')){
          digitalWrite(motorPin,0);
        }
    }
}
