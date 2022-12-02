// Radio module Part
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"

// Screen Part
#include "Adafruit_SSD1306.h"
Adafruit_SSD1306 display(-1);

enum ADC_modes
{
  ADC_A0,
  ADC_A1,
  ADC_A2,
  ADC_A3,
  ADC_A4,
  ADC_A5,
  ADC_A6,
  ADC_A7,
  ADC_SENSOR,
  ADC_1V1,
  ADC_GND,
  ADC_AREF,
  ADC_VCC,
};

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Objects
MPU6050 mpu;

//-----------------------Centrale inertielle----------------------//

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//-----------------------PIN VARIABLES----------------------//

//Buzzer part
int BUZZER = 5;
const byte interruptPin = 3;
int SW_PIN2 = 6;




//-----------------------JOYSTICK----------------------//
#define NUM_JOY 2
#define MIN_VAL 0
#define MAX_VAL 1023

const int joyPinLeft [2] = {A0, A1};
const int joyPinRight [2] = {A2, A3};
const int joyOffset = 0;
int joyValLeft [NUM_JOY] = {0, 0};
int joyValRight [NUM_JOY] = {0, 0};

struct JoystickStruct{
  int joystickGaucheX_angle_Lacet ; // int = 2 octets 
  int joystickGaucheY_angle_tanguage; 
  int joystickDroiteX_accel_tanguage;
  int joystickDroiteY_accel_rouli;  
  bool mode;
};

JoystickStruct donnees;

//-----------------------RADIO----------------------//
//Nos propres pipes pour la sécurité
const byte pipes[][6] = {"SOU01","KEV02"};
// 0xE8E8F0F0A1LL;
//const uint64_t pipe02 = 0xE8E8F0F0A2LL;
RF24 radio(8,9);  //CE, CSN

bool stateScreen = false;

//-----------------------ADC----------------------//


void ADC_startConvert();
unsigned int ADC_read();
//Fonction qui teste si l'ADC est en train de convertir ou non
boolean ADC_available();
void ADC_enable();
void setAnalogMux(ADC_modes mux);
//Fonction qui modifie la résolution de l'ADC
void ADC_setResolution(boolean res);
void ADC_setPrescaler(byte prescl);

unsigned int NewAnalogRead(ADC_modes mode)
{
  setAnalogMux(mode);
  ADC_startConvert();
  while (!ADC_available());
  {
    return ADC_read(); 
  }
}

void interrupt_routine()
{
  if(digitalRead(SW_PIN2)==0)
    stateScreen = !stateScreen;
}

//-----------------------CONVERSION MAP JOYSTICK----------------------//
float joyRawToPhys(int raw) { 
  ////Joystick conversion rule
  float phys = map(raw, MIN_VAL, MAX_VAL, -100 + joyOffset, 100 + joyOffset) - joyOffset; //Convertit la valeur des joystick en une valeur qui oscille entre - 100 et 100
  return phys;
}

/*
int MPU_map(int mpu_val)
{
  return (int)map(mpu_val,-120,120,-20,20);
}*/


////////////-------------------------------------------MODE JOYSTICK -----------------------------------------------/////////////
void JoystickMode()
{
      float dataJoystickAll[4];
      //-----------------------Joystick left----------------------//
      joyValLeft[0] = NewAnalogRead(ADC_A0);
      joyValLeft[1] = NewAnalogRead(ADC_A1);
      for (int i = 0; i < NUM_JOY; i++) {  
          dataJoystickAll[i] = joyRawToPhys(joyValLeft[i]); 
      }

      //-----------------------Joystick right----------------------//
      joyValRight[0] = NewAnalogRead(ADC_A2);
      joyValRight[1] = NewAnalogRead(ADC_A3);
      for (int i = 0; i < NUM_JOY; i++) {
          dataJoystickAll[i+2] = joyRawToPhys(joyValRight[i]); 
      }
      
      //-------------------Ajout des données dans une structure --------//
      donnees.joystickGaucheX_angle_Lacet =dataJoystickAll[0];
      donnees.joystickGaucheY_angle_tanguage=dataJoystickAll[1];
      donnees.joystickDroiteX_accel_tanguage=dataJoystickAll[2];
      donnees.joystickDroiteY_accel_rouli=dataJoystickAll[3];
      //-------------------Affichage sur l'OLED ---------------//
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("X1 =");
      display.setCursor(25,0);
      display.print(donnees.joystickGaucheX_angle_Lacet);
      display.setCursor(60,0);
      display.print("Y1 =");
      display.setCursor(85,0);
      display.print(donnees.joystickGaucheY_angle_tanguage);
      display.setCursor(0,20);
      display.print("X2 =");
      display.setCursor(25,20);
      display.print(donnees.joystickDroiteX_accel_tanguage);
      display.setCursor(60,20);
      display.print("Y2 =");
      display.setCursor(85,20);
      display.print(donnees.joystickDroiteY_accel_rouli);
      display.display(); 
}

////////////-------------------------------------------MODE MPU -----------------------------------------------/////////////

void MPU_mode()
{
  //if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  //delay(100); //Slow down
    
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    ////Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                // //Serial.print("ypr\t");
                // //Serial.print(ypr[0] * 180/M_PI);
                // //Serial.print("\t");
                // //Serial.print(ypr[1] * 180/M_PI);
                // //Serial.print("\t");
                // //Serial.println(ypr[2] * 180/M_PI);
        donnees.joystickGaucheX_angle_Lacet = ypr[0] * 180/M_PI;
        donnees.joystickGaucheY_angle_tanguage= ypr[1] * 180/M_PI;  
      #endif
      #ifdef OUTPUT_READABLE_REALACCEL        
                  // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        donnees.joystickDroiteX_accel_tanguage=aaReal.y;
        donnees.joystickDroiteY_accel_rouli=aaReal.z;
      #endif
      //Affichage OLED

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("La.:");
      display.setCursor(25,0);
      display.print(donnees.joystickGaucheX_angle_Lacet);
      display.setCursor(60,0);
      display.print("Tan.:");
      display.setCursor(85,0);
      display.print(donnees.joystickGaucheY_angle_tanguage);
      display.setCursor(0,20);
      display.print("aT.:");
      display.setCursor(25,20);
      display.print(donnees.joystickDroiteX_accel_tanguage);
      display.setCursor(60,20);
      display.print("aR.:");
      display.setCursor(85,20);
      display.print(donnees.joystickDroiteY_accel_rouli);
      display.display(); 
    }   
}

//--------------------------------------------------------------------------SETUP---------------------------------------------------------------------------//


void setup() {

  //----------MPU--------//
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize //Serial communication
    //Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    ////Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(235);
    mpu.setYGyroOffset(85);
    mpu.setZGyroOffset(-84);
    mpu.setZAccelOffset(1688); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
  
  //Init Joystick
  for (int i = 0; i < NUM_JOY; i++) pinMode(joyPinLeft[i], INPUT);
  for (int i = 0; i < NUM_JOY; i++) pinMode(joyPinRight[i], INPUT);
   pinMode(SW_PIN2, INPUT_PULLUP);
  //Activation de l'ADC
  ADC_enable();
  //Fonction qui choisit correctement la fréquence d’échantillonnage de notre ADC
  ADC_setPrescaler(32);
  //Fonction qui sélectionne l'une des trois tensions de référence => ici VCC (5V)
  ADC_setReference(ADC_VCC);
  //Fonction qui sélectionne un canal auquel le microphone est attaché => ICI le pin A0
  //setAnalogMux(ADC_A0);
  //Fonction qui modifie la résolution de l'ADC
  ADC_setResolution(false);
  //Lecture de la valeur
  NewAnalogRead(ADC_A0); 

  //Affichage OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();   
  display.setTextSize(1);
  display.print("T");
  display.display();

  //Buzzer
  pinMode(BUZZER, OUTPUT);

  //RADIO
  radio.begin();
  radio.setChannel(111); 
  delay(2);
  radio.setDataRate(RF24_1MBPS); //Quantité de données envoyées par seconde
  radio.setPALevel(RF24_PA_LOW); //Distance
  radio.openWritingPipe(pipes[0]); //Pipe réservé à l'écriture
  radio.openReadingPipe(1, pipes[1]); //Pipe réservé à la lecture
  radio.enableAckPayload(); //for autoanswers
  radio.enableDynamicPayloads(); //must have for multi pipe receiving
  
  radio.startListening();

  //Interruption
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), interrupt_routine, RISING);

}

float data[2] = {0.1,1.4};

//------------------------------------------------------------------------------------------LOOP-------------------------------------------------------------------------------------------------//
void loop() {
  donnees.mode = stateScreen;
  if(stateScreen == true)
  {    
    ////Serial.println("Mode 1");
    JoystickMode();   
  }
  else if(stateScreen == false)
  {
    //Serial.println("Mode 2");
    MPU_mode();
  } 

   //-----------------------Ecriture radio----------------------//

      
      radio.stopListening();
      radio.write(&donnees, sizeof(donnees));
      delay(20);
      radio.startListening();
      
      data[0] = 0.1;
      if(radio.available()) //Si on reçoit des données
      {          
        radio.read(&data, sizeof(data)); //Lecture des données

        if(data[0] == 4.0)
        {
          tone(BUZZER, 600);
          delay(30);
          noTone(BUZZER);   
        }
             
      }
      

}

void setAnalogMux(ADC_modes mux){
  cli();
  switch(mux){
    case ADC_A0: // ADC_A0 (default)
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
        break;
    case ADC_A1: //ADC_A1
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
        ADMUX |= (1 << MUX0);
        break;
    case ADC_A2: //ADC_A2
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX2) | (1 << MUX0));
        ADMUX |= (1 << MUX1);
        break;
    case ADC_A3: //ADC_A3
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX2));
        ADMUX |= ((1 << MUX1) | (1 << MUX0));
        break;
    case ADC_A4: //ADC_A4
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX1) | (1 << MUX0));
        ADMUX |= (1 << MUX2);
        break;
    case ADC_A5: //ADC_A5
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX1));
        ADMUX |= ((1 << MUX2) | (1 << MUX0));
        break;
    case ADC_A6: //ADC_A6
        ADMUX &= ~ ((1 << MUX3) | (1 << MUX0));
        ADMUX |= ((1 << MUX2) | (1 << MUX1));
        break;
    case ADC_A7: //ADC_A7
        ADMUX &= ~ (1 << MUX3);
        ADMUX |= ((1 << MUX2) | (1 << MUX1) | (1 << MUX0));
        break;
    case ADC_SENSOR: //ADC_SENSOR
        ADMUX &= ~ ((1 << MUX2) | (1 << MUX1) | (1 << MUX0));
        ADMUX |= (1 << MUX3);
        break;
    case ADC_1V1: //ADC_1V1
        ADMUX &= ~ (1 << MUX0);
        ADMUX |= ((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
        break;
    case ADC_GND: //ADC_GND 
        ADMUX |= ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
        break;
  }
  sei();
}

//Fonction qui sélectionne l'une des trois tensions de référence
void ADC_setReference(ADC_modes ref){
    cli();
    switch (ref) {
    case ADC_1V1: // 1v1
        ADMUX |= ((1 << REFS1) | (1 << REFS0));
        break;
    case ADC_AREF: // ADC_AREF (default)
        ADMUX &= ~((1 << REFS1) | (1 << REFS0));
        break;
    case ADC_VCC: // ADC_VCC
        ADMUX &= ~(1 << REFS1);
        ADMUX |= (1 << REFS0);
        break;
    }
    sei();
}

//Fonction qui modifie la résolution de l'ADC
void ADC_setResolution(boolean res)
{
  cli();
  if(res==true)
  {
    ADMUX |= (1<<ADLAR); // on met le bit ADLAR à 1, résolution à 8 bits
  }
  else
  {
    ADMUX &= ~(1<<ADLAR);// on met le bit ADLAR à 0, résolution à 10 bits
  }
  sei();
}

void ADC_enable()
{
  cli();
  ADCSRA |= (1 << ADEN); //Active l'ADC, on met le bit ADEN à 1
  sei(); 
}

boolean ADC_available() { 
    if (ADCSRA & (1 << ADSC)) { // si le bit ADSC est à 1, conversion en cours
        return false;
    }
    else { // sinon l'ADC est disponible
        return true;
    }
}

void ADC_startConvert() { 
    cli();
    ADCSRA |= (1 << ADSC); // On met le bit ADSC à 1 à chaque nouvelle conversion
    sei();
}

//récupération de la valeur de l'ADC codée sur 10 bits
unsigned int ADC_read() { 
    return ADC;
}


//Fonction qui choisit correctement la fréquence d’échantillonnage de notre ADC
void ADC_setPrescaler(byte prescl){
    cli();
    switch (prescl) {
    case 2: // (default)
        ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
        break;
    case 4:
        ADCSRA &= ~((1 << ADPS2) | (1 << ADPS0));
        ADCSRA |= (1 << ADPS1);
        break;
    case 8:
        ADCSRA &= ~ (1 << ADPS2);
        ADCSRA |= ((1 << ADPS1) | (1 << ADPS0));
        break;
    case 16:
        ADCSRA &= ~((1 << ADPS1) | (1 << ADPS0));
        ADCSRA |= (1 << ADPS2);
        break;
    case 32:
        ADCSRA &= ~ (1 << ADPS1);
        ADCSRA |= ((1 << ADPS2) | (1 << ADPS0));
        break;
    case 64:
        ADCSRA &= ~ (1 << ADPS0);
        ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));
        break;
    case 128:
        ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
        break;
    }
    sei();
}
