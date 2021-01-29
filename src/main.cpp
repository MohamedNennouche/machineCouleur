// bibliothèque à inclure
#include <Arduino.h>

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Model.h>
#include "constants.h"
using namespace std;

/* UNO : 
     Connecte SCL    à A5
     Connecte SDA    à A4
   MEGA : 
     Connecte SCL    à 21
     Connecte SDA    à 20
   ESP32 :
     Connecte SCL    à D1
     Connecte SDA    à D2
   Connecte VDD      à la sortie 3.3V
   Connecte GROUND   au GND
   Connecte INT      à la pin 2
   Connecte le Servo à la pin 3*/

/*
 * *******************************************************************************************************************************************************************************************
 */


/*
 * Sources : 
 * CAPTEUR :
 * https://github.com/systembolaget/Physical-computing-sensor-servo-tutorial-6a-Colour-finder-with-ams-TCS34725-and-HD-1900A
 * 
 * MODELE IA : 
 * https://github.com/AbdelazizBouzidi/MOON_MACHINE
 */


 /*
 * *******************************************************************************************************************************************************************************************
 */

// Les constante
const byte interruptPin = 2; // Sortie INT du capteur TCS34725
/*
 * tableaux contenant les couleurs sous cette ordre : r,g,b,angle (angle en microsecondes) à tirer avec les tests dans différents environnements et en tirer une moyenne
 */
const int SAMPLES[] = {665, 755, 850, 950, 1050, 1155, 1265, 1370, 1475, 1585};

// Servo 
const int monServo = 3;
/*
 * Définition du capteur de couleur et du servo
 */
Adafruit_TCS34725 SENSOR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Servo SERVOHD1900A;

/*
 * *******************************************************************************************************************************************************************************************
 */

// Variables
unsigned long timeNow = 0; // horodatage à rafrachir à chaque loop
const byte timeInterval = 5; // interval pour le servo (peut changer avec les tests)
int angleCurrent; // ce sera pris en microsecondes pour optimiser
int angleTarget; // ce sera pris en microsecondes pour optimiser
uint16_t redSensor, greenSensor, blueSensor, clearSensor; // Pour receuillir les données du capteurs r,g,b,c
volatile boolean state = false; // Servira pour l'interruption 

// Pour le compilateur, s'active quand une interruption intervient
void isr()
{
  state = true;
}


/*
 * *******************************************************************************************************************************************************************************************
 */

/*
 * DEBUT DU PROGRAMME
 */


void setup()
{
  // L'interruption TCS34725 active à l'état bas et drain ouvert
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  //Initialisation du capteur
  if (SENSOR.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  // On génère une interruption à chaque cycle
  SENSOR.write8(TCS34725_PERS, TCS34725_PERS_NONE);
  // Activer la fonction d'interruption du capteur
  SENSOR.setInterrupt(true);
  

  // Initialisation du Servo
  SERVOHD1900A.attach(monServo);
  // Repositionner à la position initiale (angle 0° en microsecondes)
  SERVOHD1900A.writeMicroseconds(600);

  // Chaque couleur correspond à un angle 
  angleCurrent = 0;
  angleTarget = 600;
}


/*
 * *******************************************************************************************************************************************************************************************
 */

/*
 * DEBUT DU LOOP 
 */

/*
 * *******************************************************************************************************************************************************************************************
 */

void loop()
{
  /*
   * Fonction de lecture du capteur 
   * Elle permet de ne pas interrompre le cours de la boucle
   */
  readSensor();
  // Initialisation du modèle
  ModelInit Model1(3, 12, 9, WB::W, WB::B);
  // Identification entre la couleur et l'angle (à MODIFIER avec les tests)
  angleTarget = SAMPLES[Model1.predict(redSensor, greenSensor, blueSensor)];
  // Fonction pour la rotation du servo en microsecondes
  rotateServo();
}


/*
 * *******************************************************************************************************************************************************************************************
 */

/*
 * LES FONCTIONS
 */

 /*
 * *******************************************************************************************************************************************************************************************
 */

void readSensor()
{
  if (state) // A chaque interruption
  {
    // On lit
    getRawData_noDelay(&redSensor, &greenSensor, &blueSensor, &clearSensor);
    // Désactiver l'interruption après lecture
    SENSOR.clearInterrupt();
    // réactualisation de la variable state
    state = false;
  }
}

void rotateServo()
{
  // S'assurer qu'il y a pas de chevauchement entre deux rotations pour une couleur (intervale à choisir avec les tests)
  if (millis() - timeNow >= timeInterval)
  {
    // réintialiser le timeNow (pour chaque loop)
    timeNow = millis();
    // Si on n'a pas encore atteint l'angle on ajoute (c'est en microseondes donc très précis)  --> l'angle est définit dans la fonction identifySample()
    if (angleCurrent != angleTarget)
    {
      if (angleCurrent <= angleTarget)
      {
        angleCurrent += 5;
        SERVOHD1900A.writeMicroseconds(angleCurrent);
      }
      else
      {
        if (angleCurrent >= angleTarget)
        {
          angleCurrent -= 5;
          SERVOHD1900A.writeMicroseconds(angleCurrent);
        }
      }
    }
  }
}

void getRawData_noDelay(uint16_t *redSensor, uint16_t *greenSensor, uint16_t *blueSensor, uint16_t *clearSensor)
{
  /*
   * L'intérêt de cette fonction au lieu de la traditionnelle getRawData() est d'enlever les delay qui bloquent
   * le code tout en utilisant la méthode des interruptions en utilisant la fonction read16)
   */
  *clearSensor = SENSOR.read16(TCS34725_CDATAL);
  *redSensor = SENSOR.read16(TCS34725_RDATAL);
  *greenSensor = SENSOR.read16(TCS34725_GDATAL);
  *blueSensor = SENSOR.read16(TCS34725_BDATAL);
}
