#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>

Servo servoGauche;
Servo servoDroite;

float data[2]={4.0,0.6};
//int data = 14;

//const uint64_t pipe01 = 0xE8E8F0F0A1LL;
//const uint64_t pipe02 = 0xE8E8F0F0A2LL;

struct JoystickStruct{
    int joystickGaucheX_angle_Lacet;    // int = 4 octets 
    int joystickGaucheY_angle_tanguage; 
    int joystickDroiteX_accel_tanguage;
    int joystickDroiteY_accel_rouli;
    bool mode;
};


JoystickStruct donnees;

const byte pipes[][6] = {"SOU01","KEV02"};

RF24 radio(9,10);  // CE, CSN

void setup() {
  Serial.begin(9600);
  servoGauche.attach(6);
  servoDroite.attach(2);
  //pinMode(2,OUTPUT);
  radio.begin();
  radio.setChannel(111);
  donnees.joystickGaucheX_angle_Lacet=0;
  donnees.joystickGaucheY_angle_tanguage=0;
  donnees.joystickDroiteX_accel_tanguage=0;
  donnees.joystickDroiteY_accel_rouli=0;
  delay(2);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  //radio.stopListening();
      
  radio.enableAckPayload(); //for autoanswers

  radio.enableDynamicPayloads(); //must have for multi pipe receiving

  radio.startListening();

  Serial.print("Initialyze done!");
}

//////------------------MAP de conversion -----------------/////////

int joystick_conversion(int dataJoy)
{
  int data2 = map(dataJoy,-100,100,0,180);
  return data2;
}

int mpu_angle_conversion(int data)
{
  int data2 = map(data,-150,150,0,180);
  return data2;
}

int mpu_accel_conversion(int data)
{
  int data2 = map(data,-2000,1000,0,180);
  return data2;
}


void loop() {
 
  
  //Envoie des données
  radio.stopListening();     
  radio.write(&data, sizeof(data));
  radio.startListening();
  delay(50);
  
  //Récéption des données
  if (radio.available()) {
    Serial.println("Received :");
    radio.read(&donnees, sizeof(donnees)); //Lecture des données

    if(donnees.mode == true) //Mode joystick
    {
      Serial.println("mode Joystick :");
      //Actionner les servomoteurs
      servoGauche.write(joystick_conversion(donnees.joystickGaucheX_angle_Lacet));
      delay(60);
      servoDroite.write(joystick_conversion(donnees.joystickDroiteX_accel_tanguage));
      delay(60);
    }
    else if(donnees.mode == false)
    {
       Serial.println("mode MPU :");
      //Actionner les servomoteurs
      servoGauche.write(mpu_angle_conversion(donnees.joystickGaucheX_angle_Lacet));
      delay(60);
      servoDroite.write(mpu_accel_conversion(donnees.joystickDroiteX_accel_tanguage));
      delay(60);
    }
    Serial.print("Donnée 1: ");
    Serial.print(donnees.joystickGaucheX_angle_Lacet);
    Serial.print("Donnée 2: ");
    Serial.println(donnees.joystickGaucheY_angle_tanguage);
    Serial.print("Donnée 3: ");
    Serial.print(donnees.joystickDroiteX_accel_tanguage);
    Serial.print("Donnée 4: ");
    Serial.println(donnees.joystickDroiteY_accel_rouli);
  }

  //Ecriture serial
    
  
}
