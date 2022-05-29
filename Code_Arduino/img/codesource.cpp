/* File:   main.cpp
 * Author: Quentin Surdez
 *
 * Created on 05 may 2022, 08:01
 */
/****************************************************************************************
 *** Programme de régulation PID v1.0 ***
 *** ATmega4809 ***
 *** Numérique ***
 *** Auteur : SURDEZ Quentin ***
 *** Date : 05/05/2022 ***
 *** Description: Ce programme gère la régulation du capteur par un régulateur de type PID
 ****************************************************************************************/

/*****************************
 * Intégration des librairies *
 ******************************/
#include <Arduino.h>
#include <Wire.h>


/*****************************
 * Câblaage *
 *
 * Entrées : 2, 9, 21, 20, 17
 *
 * Sorties num : 3, 4, 8, 9
 * Sorties PWM : 5, 6
 ******************************/

/*****************************
 * Définition des pins utilisées sur NanoEvery *
 ******************************/
#define _DEBUG false
#define In1 3
#define In2 4
#define In3 7
#define In4 8
#define Enable1 5
#define Enable2 6
#define Sw_1 21
#define Sw_2 20
#define photoElectricSensor 2
#define photoElectricSensor1 9
#define USSensor 17
#define pin_PID 10
#define stop_PIN A7

/****************************************************************************************
 * Déclaration des variables *
 ****************************************************************************************/    
unsigned int tick_codeuse = 0;     // Compteur de tick de la codeuse du moteur A
unsigned int tick_codeuse1 = 0;   // Compteur de tick de la codeuse du moteur B
const int frequence_echantillonnage = 250;  // Fréquence du pid
const int dt = 4;                          // Période en millisecondes
const int tick_par_tour_codeuse = 24;
float consigne_moteur = 3;  //  Nombre de tours de roue par seconde
float consigne_moteur1 = 3;
float erreur_old = consigne_moteur;   // Initialisation de l'ancienne erreur
float erreur_old1 = consigne_moteur;
float somme_erreur = 0;   // Somme des erreurs pour l'intégrateur
float somme_erreur1 = 0;
bool Interruptions = false;
double valeur[500];
unsigned int tick_distance = 0;
unsigned int tick_distance1 = 0;
bool enAvant = 1;
bool tourner = 0;

/*--  Paramètres PID ---------------------------------------- */
float kp = 30;           // Coefficient proportionnel
float ki = 0.01;           // Coefficient intégrateur
float kd = 0.001;           // Coefficient dérivateur
double controleur = 0;
double controleur1 = 0;

/****************************************************************************************
 * Déclaration des fonctions  															*
 ****************************************************************************************/
void compteur();
void compteur1();
void asservissement();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void compteurDistance();
void compteurDistance1();
void receiveEvent(int howMany);
void cinqMenAvant();
void tourneSurSoi();
void toutDroit();



// Communication avec le Raspberry via I2C -------------------------------------------------------------------
void receiveEvent(int howMany){
  while(Wire.available()){
    int c = Wire.read();
/*     Serial.print("Ordre ");
    Serial.println(c); */

    switch (c)
    {
    case 3:
      cinqMenAvant();
    break;
    case 4:
      tourneSurSoi();
    break;
    case 10:
      toutDroit();
    break;
    }
  }
}

// Set up du programme 5m en avant -------------------------------------------------------------------
void cinqMenAvant(){
  
/*   // On enlève les interruptions attachées aux fonctions de base
  detachInterrupt(digitalPinToInterrupt(photoElectricSensor));
  detachInterrupt(digitalPinToInterrupt(photoElectricSensor1)); */
  tick_distance=0;
  tick_distance1=0;

  // On remet les interrptions, mais avec les fonctions qui nous intéressent pour ce programme
  attachInterrupt(digitalPinToInterrupt(photoElectricSensor), compteurDistance, RISING);
  attachInterrupt(digitalPinToInterrupt(photoElectricSensor1), compteurDistance1, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_PID), asservissement, RISING);  // Interruption pour calcul du PID et asservissement

  //On donne une consigne petite pour être sûr que le robot va tout droit
  consigne_moteur = 2;
  consigne_moteur1  = 2;
  enAvant = 1;

}
// Set up du programme tourner sur soi -------------------------------------------------------------------
void tourneSurSoi(){

/*   //// On enlève les interruptions attachées aux fonctions de base
  detachInterrupt(digitalPinToInterrupt(photoElectricSensor));
  detachInterrupt(digitalPinToInterrupt(photoElectricSensor1)); */


  // On remet les interrptions, mais avec les fonctions qui nous intéressent pour ce programme
  attachInterrupt(digitalPinToInterrupt(photoElectricSensor), compteurDistance, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_PID), asservissement, RISING);  // Interruption pour calcul du PID et asservissement

  //On donne une consigne petite pour être sûr que le robot fasse ce qu'on lui demande
  consigne_moteur = 2;
  consigne_moteur1 = 0;

  tourner=1;

}

// Set up du programme tout droit -------------------------------------------------------------------
void toutDroit(){

    /****************************************************************************************
   * Interruptions															*
   ****************************************************************************************/     
    attachInterrupt(digitalPinToInterrupt(photoElectricSensor), compteur, RISING);   // Interruption sur tick de la codeuse moteur A
    attachInterrupt(digitalPinToInterrupt(photoElectricSensor1), compteur1, RISING); // Interruption sur tick de la codeuse moteur B
    attachInterrupt(digitalPinToInterrupt(pin_PID), asservissement, RISING);  // Interruption pour calcul du PID et asservissement


}



// Interruptions sur les capteurs -------------------------------------------------------------------
void compteur(){
    tick_codeuse++;  // On incrémente le nombre de tick de la codeuse

}

void compteur1(){
  tick_codeuse1++;
}


// Interruptions sur les capteurs avec limite de distance -------------------------------------------------------------------
void compteurDistance(){
    tick_codeuse++, tick_distance++;
}

void compteurDistance1(){
    tick_codeuse1++;
    tick_distance1++; 
}


// Commande moteurs -------------------------------------------------------------------
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1) // SENS HORAIRE
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) // SENS ANTI-HORAIRE
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
 
// Fonction appelée dans l'interruption timée pour réguler la vitesse des moteurs ------------------------------------
void asservissement()
{

   int tick = tick_codeuse;
   int tick1 = tick_codeuse1;
   
 /*On sait que le compteur doit s'incrémenter tour_roue_codeuse fois pour faire un tour. La fréquence fois les ticks divisé par le nombre nécessaire pour faire un tour nous donne la vitesse*/
  int frequence_codeuse = frequence_echantillonnage * tick;
  int frequence_codeuse1 = frequence_echantillonnage * tick1;
  float vitesse = frequence_codeuse/tick_par_tour_codeuse;
  float vitesse1 = frequence_codeuse1/tick_par_tour_codeuse;
  /*Calcul de l'erreur en faisant la différence entre la consigne moins la vitesse calculée*/
  float erreur = consigne_moteur - vitesse;
  float erreur1 = consigne_moteur1 -vitesse1;

  /*Réinitialisation des ticks du capteur lumineux*/
  tick_codeuse = 0;
  tick_codeuse1 = 0;

  /*Calcul des différentes valeurs du PID*/
  double P_value = kp * erreur;
  double P_value1 = kp * erreur1;
  double I_value = somme_erreur + (ki * (((erreur + erreur_old) * dt) /2)); // Calcul de l'aire du trapèze
  double I_value1 = somme_erreur1 + (ki * (((erreur1 + erreur_old1) * dt) /2));
  double D_value = kd * ((erreur - erreur_old) / dt);
  double D_value1 = kd * ((erreur1 - erreur_old1) / dt);

  /*Calcul du contrôleur*/
   controleur = P_value + I_value + D_value;
   controleur1 = P_value1 + I_value1 + D_value1;

  /*Limites du contrôleur*/
  if (controleur < 0)
  {
    controleur = 0;
  }
  else if (controleur > 255)
  {
    controleur = 255;
  }

    if (controleur1 < 0)
  {
    controleur1 = 0;
  }
  else if (controleur1 > 255)
  {
    controleur1 = 255;
  }
    
  /*Ecriture aux moteurs*/
  setMotor(1, controleur, Enable1, In1, In2);
  setMotor(1, controleur1, Enable2, In3, In4);
  //Serial.println(controleur);
  /*Mise à niveau des valeurs*/
  somme_erreur = I_value;
  somme_erreur1 = I_value1;
  erreur_old = erreur;
  erreur_old1 = erreur1;

}


/****************************************************************************************
 * Set up fonction  															*
 ****************************************************************************************/
void setup() {
   
// Set up de la communication avec le Raspberry PI via I2C ------------------------------------
    Wire.begin(0x08);
    Wire.onReceive(receiveEvent);
   
    Serial.begin(115200);         // Initialisation port COM
    pinMode(Enable1, OUTPUT);     // Sortie moteur A
    pinMode(Enable2, OUTPUT);     // Sortie moteur B
    pinMode(photoElectricSensor, INPUT);
    pinMode(photoElectricSensor1, INPUT);
    pinMode(pin_PID, INPUT);
    pinMode(stop_PIN, INPUT);


}


void loop(){


  if(digitalRead(stop_PIN)==1 or tick_codeuse>=100 or tick_codeuse1>=100){

    setMotor(1, 0, Enable1, In1, In2);
    setMotor(1, 0, Enable2, In3, In4);
    //detachInterrupt(digitalPinToInterrupt(pin_PID));

    Interruptions = true;
  }

  if (digitalRead(stop_PIN)==0 && Interruptions==true){

    attachInterrupt(digitalPinToInterrupt(pin_PID), asservissement, RISING);

    Interruptions = false;
  }


    if (enAvant == 1){
  
    // Limite du nombre d'incrémentations pour que le robot fasse 5m depuis l'allumage
    if (tick_distance >= 580){
      setMotor(1, 0, Enable1, In1, In2);
      setMotor(1, 0, Enable2, In3, In4);
      detachInterrupt(digitalPinToInterrupt(photoElectricSensor));
      detachInterrupt(digitalPinToInterrupt(photoElectricSensor1));
      detachInterrupt(digitalPinToInterrupt(pin_PID));
      Serial.println("5m atteint");
      enAvant= 0;
    }
  }


  if (tourner){
  // Limite du nombre d'incrémentations pour que le robot fasse 5m depuis l'allumage
  if (tick_distance >= 48){
      setMotor(1, 0, Enable1, In1, In2);
      setMotor(1, 0, Enable2, In3, In4);
      detachInterrupt(digitalPinToInterrupt(photoElectricSensor));
      detachInterrupt(digitalPinToInterrupt(photoElectricSensor1));
      detachInterrupt(digitalPinToInterrupt(pin_PID));
      tourner=0;
  }
  }

}