// programme pour le Sol'Ex (solar explorer) de Christian Buil.
// version originale : Jean Brunet  le 17/01/2023 écrite pour commander le déplacement du réseau.

// Cette nouvelle version a été réécrite depuis le 22/04/2023 par Pascal Berteau avec 5 raies programmées 
// et la commande de 3 moteurs, réseau, focus caméra et focus lunette.
// Le programme a été l'objet de nombreux ajouts par Jean Brunet et Pascal Berteau pour finalement
// aboutir à celui-ci.

// La position des raies est mémorisée, ainsi que la position du focus caméra en appuyant sur Mémo.
// Le couplage des moteurs réseau et focus caméra permet de se positionner sur une raie en obtenant automatiquement la netteté.
// Ainsi il n'y a pas besoin d'intervenir manuellement sur Sol'Ex, ni sur la lunette de visée qui utilise le troisième moteur.

// Ce programme est à choisir pour un module AT24Cxx
// Si vous utilisez la mémoire EEPROM 24LC01B, prenez le programme ESP32_SolEx_24LC01B.ino

// Pascal Berteau, Jean Brunet - le 12 / 06 /2023. Ce programme peut être distribué librement.

// IMPORTANT !!
#define initmemo 0  // Mettre à 1 pour la première utilisation pour initialisation de la mémoire. Ensuite mettre initmemo à 0

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <AT24Cxx.h>  // bibliothèque à installer dans arduino

#define MOTOR2_PIN_1  32   // Blue   - 28BYJ48 pin 1
#define MOTOR2_PIN_3  33  // Pink   - 28BYJ48 pin 2
#define MOTOR2_PIN_2   25  // Yellow - 28BYJ48 pin 3
#define MOTOR2_PIN_4  26   //  Orange - 28BYJ48 pin 4

#define MOTOR3_PIN_1  27   // Blue   - 28BYJ48 pin 1
#define MOTOR3_PIN_3  14  // Pink   - 28BYJ48 pin 2
#define MOTOR3_PIN_2   12  // Yellow - 28BYJ48 pin 3
#define MOTOR3_PIN_4  13   //  Orange - 28BYJ48 pin 4

#define MOTOR_PIN_1  19   // Blue   - 28BYJ48 pin 1
#define MOTOR_PIN_3  18  // Pink   - 28BYJ48 pin 2
#define MOTOR_PIN_2   5  // Yellow - 28BYJ48 pin 3
#define MOTOR_PIN_4  17   //  Orange - 28BYJ48 pin 4


int address = 0x50;

AT24Cxx eep(address, 32); // module AT24Cxx, mettre en Ko la longueur de l'EEPROM installée. Ici 32 Ko.

// raies programmées

unsigned int RAIEHa = 1230;
unsigned int RAIENa = 1060;
unsigned int RAIEMg = 910;
unsigned int RAIEHb = 850;
unsigned int RAIECa = 670;

const char* ssid = "SolEx";
const char* password = "solex1234";  // A MODIFIER si plusieurs Sol'Ex proches
WiFiServer server(80);

String header;
const unsigned int STEPZero = 0;  

const unsigned int STEPRes1 = 5;  
const unsigned int STEPRes2 = 25;  
const unsigned int STEPRes3 = 100;  

const unsigned int STEPCam1 = 5;  
const unsigned int STEPCam2 = 25; 

const unsigned int STEPFoc1 = 5;  
const unsigned int STEPFoc2 = 25;
const unsigned int STEPFoc3 = 200;  

const unsigned int Backlash_Res = 15;  
const unsigned int Backlash_Cam = 15 + 25;  // jeux pignons (15) et jeux focus hélicoïdal. (25) A ajuster si besoin    
const unsigned int Backlash_Foc = 15;  

unsigned int STEPREL = 0;
unsigned int indice_ray_select = 0;

const unsigned long STEP_DELAY_MICROSEC = 5000;  // vitesse moteur
int cycle = 0;
int cycle2 = 0;
int cycle3 = 0;

bool direction = true;
bool dir_Cam = true;
bool dir_Foc = true;
bool dir_Res = true;

int positionRes = 0;
int positionCam = 0;
int positionFoc = 0;
unsigned long last_step_time;

bool memo = false;
bool CouplageOk = false;
/* ---------------------------
setup
-------------------------------*/
void setup() 
{
    // Initialize serial port I/O.
    // Serial.begin(115200); // seulement pour déboggage

    // Initialize motor control pins...
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(MOTOR_PIN_3, OUTPUT);
    pinMode(MOTOR_PIN_4, OUTPUT);

    pinMode(MOTOR2_PIN_1, OUTPUT);
    pinMode(MOTOR2_PIN_2, OUTPUT);
    pinMode(MOTOR2_PIN_3, OUTPUT);
    pinMode(MOTOR2_PIN_4, OUTPUT);

    pinMode(MOTOR3_PIN_1, OUTPUT);
    pinMode(MOTOR3_PIN_2, OUTPUT);
    pinMode(MOTOR3_PIN_3, OUTPUT);
    pinMode(MOTOR3_PIN_4, OUTPUT);

    direction = true;
    dir_Cam = true;
    dir_Res = true;
    dir_Foc = true;
    
    last_step_time = 0L;
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    server.begin();
    // lecture en mémoire EEPROM de la dernière position enregistrée.
    #if initmemo
     for(int i = 0 ; i < 30; i++){   // met la mémoire à 0 pour la première utilisation. Remettre initmemo à 0 ensuite
          eep.write(i, 0);
          delay(5);
     }
    indice_ray_select = 20;  positionRes = RAIEHa; WriteMemoryRes(indice_ray_select);
    indice_ray_select = 22;  positionRes = RAIENa; WriteMemoryRes(indice_ray_select);
    indice_ray_select = 24;  positionRes = RAIEMg; WriteMemoryRes(indice_ray_select);
    indice_ray_select = 26;  positionRes = RAIEHb; WriteMemoryRes(indice_ray_select);
    indice_ray_select = 28;  positionRes = RAIECa; WriteMemoryRes(indice_ray_select);
     #endif
    byte val1 = eep.read( 0);
    byte val2 = eep.read( 1);
    positionRes = word(val1, val2);
    cycle = eep.read( 2);
    dir_Res = eep.read( 3);

    val1 = eep.read( 4);
    val2 = eep.read( 5);
    positionCam = word(val1, val2);
    cycle2 = eep.read( 6);
    dir_Cam = eep.read(7); 
     
    val1 = eep.read( 8);
    val2 = eep.read( 9);
    positionFoc = word(val1, val2);
    cycle3 = eep.read( 10);
    dir_Foc = eep.read( 11);
}
 /* --------------------------------
Boucle de traitement
-------------------------------------*/
void loop() 
{
	Wificommmand();  // affichage de la page web et traitement des retours utilisateur
}


/*---------------------------------------------------------------------------------------------
 Page web et traitement des commandes envoyées par l'utilisateur
------------------------------------------------------------------------------------------------*/
void Wificommmand()
{

	WiFiClient client = server.available();   // Listen for incoming clients
  CouplageOk = false; 
	if (client){ 
   while (client.connected()){            // loop while the client's connected
		if (client.available()) {             // if there's bytes to read from the client,
			char c = client.read();             // read a byte, then
			header += c;
			if (c == '\n') // si c'est la fin de la commande reçue
			{          
				client.println("HTTP/1.1 200 OK");
				client.println("Content-type:text/html");
				client.println("Connection: close");
				client.println();

			  if (header.indexOf("couplage") >= 0){  // couplage est coché, le moteur de la caméra est fonctionnel
					  CouplageOk = true;
				}
        // corrections manuelles de déplacement  
				if (header.indexOf("GPR") >= 0){ // Gauche / Petit Pas / Reseau
            direction = false;
            positionRes = positionRes-STEPRes1;
            commandMotor(STEPRes1,1);
				}
				else if (header.indexOf("GMR") >= 0){  // Gauche Moyen Reseau
            direction = false;
            positionRes = positionRes-STEPRes2;
            commandMotor(STEPRes2,1);
         }
         else if (header.indexOf("GGR") >= 0){ // Gauche Grand Reseau
            direction = false;
            positionRes = positionRes-STEPRes3;
            commandMotor(STEPRes3,1);
         }
				else if (header.indexOf("DPR") >= 0){    // Droit / Petit Pas / Reseau
            direction = true;
            positionRes = positionRes+STEPRes1;  
            commandMotor(STEPRes1,1);
				 }
				 else if (header.indexOf("DMR") >= 0){ // Droit / Moyen Pas / Reseau
            direction = true;
            positionRes = positionRes+STEPRes2;  
            commandMotor(STEPRes2,1);	 
				 }
          else if (header.indexOf("DGR") >= 0){ // Droit / Grand Pas / Reseau
            direction = true;
            positionRes = positionRes+STEPRes3;  
            commandMotor(STEPRes3,1);	 
				// ---------------------------
        	 } 
          else if (header.indexOf("MemoZ") >= 0){ // Mémorisation ordre 0
            positionRes = 0;
         }
         else if (header.indexOf("OrdreZ") >= 0){  // Retour à l'ordre 0           
            Goto(STEPZero,1);
				  }
          else if (header.indexOf("CamInF") >= 0){ // Focus Camera in <<
					  direction = false;
            positionCam = positionCam-STEPCam2;  
				    commandMotor(STEPCam2,2);	 
				 }
        else if (header.indexOf("CamInS") >= 0){ //Focus Camera in <
					  direction = false;
            positionCam = positionCam-STEPCam1;  
				    commandMotor(STEPCam1,2);	 
				 }
          else if (header.indexOf("CamOutF") >= 0){ //Focus Camera Out >>
            direction = true;
            positionCam = positionCam+STEPCam2;  
            commandMotor(STEPCam2,2);	 
				 }
          else if (header.indexOf("CamOutS") >= 0){ //Focus Camera Out >
            direction = true;
            positionCam = positionCam+STEPCam1;  
            commandMotor(STEPCam1,2);	 
				 } 
          else if (header.indexOf("MemoCam") >= 0){   // mémorisation de la mise au point pour la raie en cours.
          // enregistrement la position du moteur Cam pour une raie sélectionnée        
            WriteMemoryCam(indice_ray_select);
            WriteMemoryRes(indice_ray_select);
          }
         else if (header.indexOf("FocInX") >= 0) { //Focus Lunette in <<<
					   direction = false;
             positionFoc=positionFoc-STEPFoc3;  
				    commandMotor(STEPFoc3,3);	 
				 }
          else if (header.indexOf("FocInF") >= 0){ //Focus Lunette in <<
					  direction = false;
            positionFoc = positionFoc-STEPFoc2;  
				    commandMotor(STEPFoc2,3);	 
				 }
        else if (header.indexOf("FocInS") >= 0){ //Focus Lunette in <
            direction = false;
            positionFoc = positionFoc-STEPFoc1;  
            commandMotor(STEPFoc1,3);	 
				 }
          else if (header.indexOf("FocOutF") >= 0){ //Focus Lunette Out >>
            direction = true;
            positionFoc = positionFoc+STEPFoc2;  
            commandMotor(STEPFoc2,3);	 
				 }
          else if (header.indexOf("FocOutS") >= 0){ //Focus Lunette Out >
					  direction = true;
            positionFoc = positionFoc+STEPFoc1;  
				    commandMotor(STEPFoc1,3);	 
				 } 
          else if (header.indexOf("FocOutX") >= 0) { //Focus Lunette Out >>>
            direction = true;
            positionFoc=positionFoc+STEPFoc3;  
            commandMotor(STEPFoc3,3);	 
				 }
 
          else if (header.indexOf("Ca") >= 0){ // Position reseau sur raies définies  
            indice_ray_select = 28; 
            RAIECa = ReadMemoryRes(indice_ray_select);
            Goto(RAIECa,1); 
            if(CouplageOk == true){
              Goto(ReadMemoryCam(indice_ray_select),2);
            }
          }
          else if (header.indexOf("Hb") >= 0){
           indice_ray_select = 26; 
           RAIEHb = ReadMemoryRes(indice_ray_select);
           Goto(RAIEHb,1);
           if(CouplageOk == true){
            Goto(ReadMemoryCam(indice_ray_select),2); 
            }     
          }
          else if (header.indexOf("Mg") >= 0){
           indice_ray_select = 24; 
           RAIEMg = ReadMemoryRes(indice_ray_select);
           Goto(RAIEMg,1); 
           if(CouplageOk == true){
            Goto(ReadMemoryCam(indice_ray_select),2);	 
            }
          }
          else if (header.indexOf("Na") >= 0){ 
           indice_ray_select = 22; 
           RAIENa = ReadMemoryRes(indice_ray_select);
           Goto(RAIENa,1);
           if(CouplageOk == true){
             Goto(ReadMemoryCam(indice_ray_select),2);	
             }
          }
          else if (header.indexOf("Ha") >= 0){ 
            indice_ray_select = 20; 
            RAIEHa = ReadMemoryRes(indice_ray_select);
            Goto(RAIEHa,1); 
            if(CouplageOk == true){
              Goto(ReadMemoryCam(indice_ray_select),2);	
             }
          }
       
       	client.println(webPage());  // page web
				// The HTTP response ends with another blank line
      	client.println();
				client.stop();
				header = "";
        } 
		  }
		}
	}
}

/* --------------------
Affichage page web
----------------------*/
String webPage()
{
  String Ch;
  Ch = "<!DOCTYPE html><html>";
  Ch +="<head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  Ch +="<style>.bouton {background-color: #504caf; border: none; color: white; width:60px; height:50px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 15px; cursor: pointer; }"; 
  Ch +=".bouton2 {background-color: #504caf; border: none; color: white; width:105px; height:50px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 15px; cursor: pointer; }"; 
  Ch +=".boutonLite {background-color: #504caf; border: none; color: white; width:65px; height:65px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 22px; cursor: pointer; }"; 
  Ch +=".boutonLite2 {background-color: #504caf; border: none; color: white; width:50px; height:70px; margin: 2px;  padding: 10px; text-decoration: none; text-align: center; display: inline-block; font-size: 22px; cursor: pointer; }"; 
  Ch +=".texteEntree {background-color: #EEEEEE; border: none; color: black; width:165px; height:30px;  padding: 10px; text-decoration: none; text-align: left; display: inline-block; font-size: 22px; cursor: pointer; }</style>"; 
  Ch +="</head><body>";    
  
  Ch +="<div style='text-align: center '><H2>Sol'Ex</H2><form enctype='multipart/form data' method=GET>"; 
  Ch +="<H4>--- R&eacute;seau " + String(positionRes)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite2' type='submit' name='GGR' value = '<<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='GMR' value = '<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='GPR' value = '<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='DPR' value = '>' />";   
  Ch +="<input button class='boutonLite2' type='submit' name='DMR' value = '>>' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='DGR' value = '>>>' /><br>"; 
  Ch +="<input button class='bouton' type='submit' name='Ca' value = 'Ca' />";  
  Ch +="<input button class='bouton' type='submit' name='Hb' value =  'H &#946 ' />";  
  Ch +="<input button class='bouton' type='submit' name='Mg' value = 'Mg' />";  
  Ch +="<input button class='bouton' type='submit' name='Na' value = 'Na' />"; 
  Ch +="<input button class='bouton' type='submit' name='Ha' value = 'H &#945 ' /><br><br>";

  String op = ""; 
  if (CouplageOk == true){
     op = "checked= 'yes'" ;
  }  
  
  Ch +="<input button class='bouton2' type='submit' name='MemoZ' value = 'Set 0' />";
  Ch +="<input button class='bouton2' type='submit' name='OrdreZ' value = 'Ordre 0' />";  
  Ch +="<input button class='bouton2' type='submit' name='MemoCam' value = 'M&eacute;mo' /><br>"; 

  Ch +="<H4>--- Cam&eacute;ra " + String(positionCam)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite' type='submit' name='CamInF' value = '<<' />"; 
  Ch +="<input button class='boutonLite' type='submit' name='CamInS' value = '<' />"; 
  Ch +="<input button class='boutonLite' type='submit' name='CamOutS' value = '>' />";   
  Ch +="<input button class='boutonLite' type='submit' name='CamOutF' value = '>>' /><br><br>";
  Ch +="Couplage <input type='checkbox' name='couplage' " + op + " value='couplage'/><br>";
 
  Ch +="<H4>--- Focuser " + String(positionFoc)+ " --- </H4></center>";
  Ch +="<input button class='boutonLite2' type='submit' name='FocInX' value = '<<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocInF' value = '<<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocInS' value = '<' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutS' value = '>' />";   
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutF' value = '>>' />"; 
  Ch +="<input button class='boutonLite2' type='submit' name='FocOutX' value = '>>>' /><br>"; 
  Ch +="</form> </div></body></html>"; 
  return Ch;
}

/*-----------------------------------------
 Routines des 3 moteurs 28BYJ48
 -------------------------------------------*/
void commandMotor(unsigned int st,unsigned int mt)  // St nombres de pas - mt numéro moteur 0: Reseau 1:foc Cam 2: Foc Lunette
{  
  switch (mt){
      case 1:
        if (direction != dir_Res )  { st = st + Backlash_Res; }  
        dir_Res = direction;         
        break;
      case 2:
        if (direction != dir_Cam )  { st = st + Backlash_Cam; }  
        dir_Cam = direction;           
        break;
      case 3:
        if  (direction != dir_Foc )  { st = st + Backlash_Foc; }  
        dir_Foc = direction;         
        break;
  }         
  for(int i=0;i<st;i++){
		Motor(mt);
		// tempo pour fixer la vitesse du moteur
		last_step_time = micros();
		while(micros() - last_step_time < STEP_DELAY_MICROSEC){}
	}
	stop(mt);  // arrêt du moteur
	setMemory(mt); // mémorisation position moteur
}

/* ----------------------------------------------------------------------
Envoi moteur 1 à 3 (mt) sur une position (st) en nombe de pas
---------------------------------------------------------------------------*/
void Goto(unsigned int st,unsigned int mt) // St nombres de pas - mt numéro moteur 0: Reseau 1:foc Cam 2: Foc Lunette
{
   switch (mt){
      case 1:
      if(positionRes != st)  // déplacement moteur si le moteur n'est pas déjà sur la même position.
      {
        if (positionRes < st)
        {
          direction = true;
          STEPREL = st-positionRes;
          positionRes = st;
          commandMotor(STEPREL,mt);                                
        }
      else
        {
          direction = false;
          STEPREL = positionRes-st;
          positionRes = st;
          commandMotor(STEPREL,mt);  
        }
      }
      break;
    case 2:
     if(positionCam != st)  // déplacement moteur si le moteur n'est pas déjà sur la même position.
      {
      if (positionCam < st)
      {
        direction = true;
        STEPREL = st-positionCam;
        positionCam = st;
        commandMotor(STEPREL,mt);                                
      }
      else
      {
        direction = false;
        STEPREL = positionCam-st;
        positionCam = st;
        commandMotor(STEPREL,mt);  
      }
     }
     break;
   }
}


void Motor(unsigned int mt)
{
  if (direction == true){
    switch (mt){
      case 1:
        stepMotorRes();
        cycle++;
		    if (cycle == 4){ 
		    	cycle = 0;
		    } 
        break;
      case 2:
        stepMotorCam(); 
        cycle2++;
		    if (cycle2 == 4){ 
		    	cycle2 = 0;
		    }         
        break;
      case 3:
        stepMotorFoc(); 
        cycle3++;
		    if (cycle3 == 4){ 
		    	cycle3 = 0;
		    }      
        break;
    }
	} else{
    switch (mt){
      case 1:
        stepMotorRes();
        if (cycle == 0){ 
		    	cycle = 4;
	    	}
	 	    cycle--;
        break;
      case 2:
        stepMotorCam();
        if (cycle2 == 0){ 
		    	cycle2 = 4;
	    	}
	 	    cycle2--;         
        break;
      case 3:
        stepMotorFoc(); 
      if (cycle3 == 0){ 
		    	cycle3 = 4;
	    	}
	 	    cycle3--;         
        break;
    }
	}
}



void stepMotorRes() //moteur reseau
{
  switch (cycle){
    case 0: // 1010
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, HIGH);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 1: // 0110
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      digitalWrite(MOTOR_PIN_3, HIGH);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 2: // 0101
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, HIGH);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, HIGH);
      break;
    case 3: // 1001
      digitalWrite(MOTOR_PIN_1, HIGH);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, HIGH);
      break;
	}
}

void stepMotorCam() //moteur camera
{
  switch (cycle2){
    case 0: // 1010
      digitalWrite(MOTOR3_PIN_1, HIGH);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, HIGH);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
    case 1: // 0110
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, HIGH);
      digitalWrite(MOTOR3_PIN_3, HIGH);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
    case 2: // 0101
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, HIGH);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, HIGH);
      break;
    case 3: // 1001
      digitalWrite(MOTOR3_PIN_1, HIGH);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, HIGH);
      break;
  }
}


void stepMotorFoc() //moteur lunette
{
  switch (cycle3){
    case 0: // 1010
      digitalWrite(MOTOR2_PIN_1, HIGH);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, HIGH);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 1: // 0110
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, HIGH);
      digitalWrite(MOTOR2_PIN_3, HIGH);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 2: // 0101
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, HIGH);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, HIGH);
      break;
    case 3: // 1001
      digitalWrite(MOTOR2_PIN_1, HIGH);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, HIGH);
      break;
  }
}

void stop(unsigned int mt) 
{
// met le moteur à l'arrêt, sans alimentation. (limite la consommation)
  switch (mt){
    case 1:
      digitalWrite(MOTOR_PIN_1, LOW);
      digitalWrite(MOTOR_PIN_2, LOW);
      digitalWrite(MOTOR_PIN_3, LOW);
      digitalWrite(MOTOR_PIN_4, LOW);
      break;
    case 2:
      digitalWrite(MOTOR2_PIN_1, LOW);
      digitalWrite(MOTOR2_PIN_2, LOW);
      digitalWrite(MOTOR2_PIN_3, LOW);
      digitalWrite(MOTOR2_PIN_4, LOW);
      break;
    case 3:
      digitalWrite(MOTOR3_PIN_1, LOW);
      digitalWrite(MOTOR3_PIN_2, LOW);
      digitalWrite(MOTOR3_PIN_3, LOW);
      digitalWrite(MOTOR3_PIN_4, LOW);
      break;
  }
}

/* ---------------------------------------------------------------------------
Lecture / écriture de la position du réseau et de la mise au point de la caméra
-------------------------------------------------------------------------------*/
void WriteMemoryCam(unsigned int IndiceRaie)
{
  byte hi;
  byte low;
  hi  = highByte(positionCam);  // sauvegarde en eeprom de la dernière position de la caméra
  low = lowByte(positionCam);
  eep.write( IndiceRaie, hi);
  eep.write( IndiceRaie + 1, low);
}

unsigned int ReadMemoryCam(unsigned int IndiceRaie)
{
  byte dat1 = eep.read( IndiceRaie);
  byte dat2 = eep.read( IndiceRaie + 1);
  return  word(dat1, dat2);
}

void WriteMemoryRes(unsigned int IndiceRaie)
{
  byte hi;
  byte low;
  hi  = highByte(positionRes);  // sauvegarde en eeprom de la dernière position
  low = lowByte(positionRes);
  eep.write( IndiceRaie + 10, hi);
  eep.write( IndiceRaie + 11, low);
}

unsigned int ReadMemoryRes(unsigned int IndiceRaie)
{
  byte dat1 = eep.read( IndiceRaie+10);
  byte dat2 = eep.read( IndiceRaie + 11);
  return  word(dat1, dat2);
}

/* -------------------------------------------
Mise en mémoire de la position d'un moteur
-----------------------------------------------*/
void setMemory(unsigned int mt)
{
   // Mise en mémoire de la dernière position du moteur utilisé.
  byte hi;
  byte low;
  switch (mt){
    case 1:   
        hi  = highByte(positionRes);  // sauvegarde en eeprom de la dernière position
        low = lowByte(positionRes);
        eep.write( 0, hi);
        eep.write( 1, low);
        eep.write( 2, cycle);
        eep.write( 3, dir_Res);
        break;
      case 2:     
        hi  = highByte(positionCam);  // sauvegarde en eeprom de la dernière position
        low = lowByte(positionCam);
        eep.write( 4, hi);
        eep.write( 5, low);
        eep.write( 6, cycle2);
        eep.write( 7, dir_Cam);
        break;
    case 3: 
        hi  = highByte(positionFoc);  // sauvegarde en eeprom de la dernière position
        low = lowByte(positionFoc);
        eep.write( 8, hi);
        eep.write( 9, low);
        eep.write( 10, cycle3);
        eep.write( 11, dir_Foc);  
        break;
    }
}

