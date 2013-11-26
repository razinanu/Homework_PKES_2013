#include <math.h>
#include <util/delay.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "Flydurino.h"

void* operator new(size_t s, void* ptr) {return ptr;}
void setup();
void loop();
int8_t checkButtons();
void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z);
int8_t linearizeDistance(int distanceRaw);
void displayDistance (int8_t dist);
int readADC(int8_t channel);
void writetoDisplay(char digit1, char digit2, char digit3);

char flydurinoPtr[sizeof(Flydurino)];
// aktuelle Beschleunigungswerte, Kompassmessungen
int16_t acc_x, acc_y, acc_z;
int16_t ori_x, ori_y, ori_z;
// Wasserwaage oder Distanzmessung
int8_t modus;
// Kanal des ADC Wandlers
int8_t channel = 3;

enum {
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Z9, MINUS_1, MINUS, SPACE
};

enum { LEFT,RIGHT,MIDDLE};
enum { DOWN_Y,MIDDLE_Y,UP_Y};

// the setup routine runs once when you press reset:
void setup() {                
  // initialize serial communication
  Serial.begin(9600);
  new(flydurinoPtr) Flydurino;

  Serial.println("----------------------------------"    );
  Serial.println("PKES Wintersemester 2013/14"           );
  Serial.println("Vorlage 2. Aufgabe "                   );
  Serial.println("----------------------------------\r\n");

  // -------------------------------------------------------------
  // Single Onboard LED configuration
  // -------------------------------------------------------------
  // set leds LED_X as output
  // alternative
  //     - intermediate
  //       DDRA=1+2+4+8;
  //     - using processor makros
  //       DDRA=((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3));
  //     - using hardware specific makros
  DDRB |= (1<<DDA7);
  // disable leds
  PORTB &= ~(1<<7);
  // -------------------------------------------------------------
  // Serial bus lines
  // -------------------------------------------------------------
  // Pin 5 = PORT E 3 = clock
  DDRE |= (1<<DDE3);
  // Pin 6 = PORT H 3 = data
  DDRH |= (1<<DDH3);
  // Pin 7 = PORT H 4 = enable
  DDRH |= (1<<DDH4);
  // -------------------------------------------------------------
  // Button configuration
  // -------------------------------------------------------------
  // not necessary but for completion
  // S1 as input
  DDRF &=~(1<<DDG4);
  // S2 as input
  DDRG &=~(1<<DDG5);

  // Configuration of ADC
  // -----------------------------------------------------
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADMUX = (1<<REFS0) | (1<<REFS0); // Use Internal Reference Voltage
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F); // choose Channel without changing other bits

  /*Dummy Readout*/
  ADCSRA |= (1<<ADSC);
  while(ADCSRA & (1<<ADSC)){}
  (void)ADCW;
  // -----------------------------------------------------

  // Configure buttons
  // -----------------------------------------------------


  // -----------------------------------------------------
  modus=0;

}

// the loop routine runs over and over again forever:
void loop() {
  if (modus==1){
      // ----------------------------------------------------------------
      // level
      // ----------------------------------------------------------------
      // Receive acceleromation values
      ((Flydurino*)flydurinoPtr)->getAcceleration(&acc_x, &acc_y, &acc_z);
      // Get compass data
      ((Flydurino*)flydurinoPtr)->getOrientation(&ori_x, &ori_y, &ori_z);
      
      Serial.print(acc_x); Serial.print("\t");
      Serial.print(acc_y); Serial.print("\t");
      Serial.print(acc_z); Serial.print("|\t");
      Serial.print(ori_x); Serial.print("\t");
      Serial.print(ori_y); Serial.print("\t");
      Serial.print(ori_z); Serial.print("|\r\n");
      displaySpiritLevel(acc_x, acc_y, acc_z);
    }

  if (modus==2){
      int8_t distance;
      // ----------------------------------------------------------------
      // Distance sensor
      // ----------------------------------------------------------------
      int distanceVolt = readADC(channel);
      float volt = (float)((5/2)*distanceVolt)/1023;
      Serial.print("Volt: ");Serial.print(volt);Serial.print(" |\t");
      Serial.print("digiVolt: ");Serial.print(distanceVolt);Serial.print(" |\t");
      distance = linearizeDistance(distanceVolt);
      Serial.print("distanceCM: ");Serial.print(distance);Serial.print("\r\n");
      displayDistance (distance);

    }

  modus=checkButtons();
}

int8_t checkButtons(){
  //int8_t modus=2;
  // Abfrage der Buttons und Moduswechsel
  // -----------------------------------------------------
  if(PINF&(1<<4)){
      modus=2;
    }
  if(PING&(1<<5)){
      modus=1;
    }

  // -----------------------------------------------------
  return modus;
}


void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
  //   3 cases for roll and pitch
  // -15 Grad <= alpha,
  // -15 Grad <= alpha  <= 15 Grad
  //  15 Grad <= alpha
  // -----------------------------------------------------
  char charsToDisplay[] = {0b00000000, 0b00000000, 0b00000000};
  char display_x[] = {0b00001100, 0b01100000, 0b01101100};
  char display_y[] = {0b0010000,0b0000010,0b10000000};
  //writetoDisplay(display_x[1], display_x[2], display_x[2]);
  int factor = 1500;
  if(acc_x>2*factor){
      charsToDisplay[0]=display_x[LEFT];
  }
  if(acc_x>1*factor && acc_x < 2*factor){
      charsToDisplay[0]=display_x[RIGHT];
  }
  if(abs(acc_x)<1*factor ){
      charsToDisplay[1]=display_x[MIDDLE];
  }
  if(acc_x<-1*factor && acc_x > -2*factor){
      charsToDisplay[2]=display_x[LEFT];
  }
  if(acc_x < -2*factor){
      charsToDisplay[2]=display_x[RIGHT];
  }

  if(acc_y>2*factor){
      charsToDisplay[0]|=display_y[DOWN_Y];
      charsToDisplay[1]|=display_y[DOWN_Y];
      charsToDisplay[2]|=display_y[DOWN_Y];
  }
  if(abs(acc_y)<2*factor ){
      charsToDisplay[0]|=display_y[MIDDLE_Y];
      charsToDisplay[1]|=display_y[MIDDLE_Y];
      charsToDisplay[2]|=display_y[MIDDLE_Y];
  }
  if(acc_y < -2*factor){
      charsToDisplay[0]|=display_y[UP_Y];
      charsToDisplay[1]|=display_y[UP_Y];
      charsToDisplay[2]|=display_y[UP_Y];
  }
  writetoDisplay(charsToDisplay[0],charsToDisplay[1],charsToDisplay[2]);
  // -----------------------------------------------------
}


int8_t linearizeDistance(int distance_raw){
  int8_t distance_cm=0;
  // Transformation der Spannungsbezogenen Distanzwerte in
  // eine Entfernung in cm
  // -----------------------------------------------------
  distance_cm = ((3500/(distance_raw+4))-1);
  // -----------------------------------------------------
  return distance_cm;
}

void displayDistance (int8_t dist)
{
  // Darstellung der Distanz in cm auf dem Display
  // -----------------------------------------------------
  char display[] = { 0b11111100, 0b01100000, 0b11011010, 0b11110010,
                     0b01100110, 0b10110110, 0b10111110, 0b11100000, 0b11111110,
                     0b11110110, 0b01100010, 0b00000010, 0b00000000 };
  int8_t rest = dist;
  int minus = 0;
  int i = 0;

  int digit[3] = { SPACE, SPACE, SPACE };

  if (rest < 0) {
      rest = abs(rest);
      minus = 1;
    }

  if (rest == 0) {
      digit[0] = 0;
    }

  while (rest > 0) {
      digit[i] = rest % 10;
      rest = rest - digit[i];
      rest = rest / 10;
      i++;
    }

  if (minus) {
      if (digit[2] == SPACE)
        digit[2] = MINUS;
      else
        digit[2] = MINUS_1;
    }

  writetoDisplay(display[digit[2]], display[digit[1]], display[digit[0]]);
  // -----------------------------------------------------
}

int readADC (int8_t channel){
  int distance_raw=255;
  // möglicherweise mehrmaliges Lesen des ADC Kanals
  // Mittelwertbildung
  // -----------------------------------------------------

  // choose channel without changing other bits
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC) ) {}
  distance_raw = ADCW;
  // ADC auslesen und zurückgeben

  // -----------------------------------------------------
  return distance_raw;
}

void writetoDisplay(char digit1, char digit2, char digit3){

  char stream[36];
  stream[0]=1;
  int i;
  for ( i=1; i<36; i++ ) {
      stream[i]=0;
    }

  for ( i=0; i<8; i++ ) {
      if (digit1 & (1<<(7-i))) stream[i+ 1]=1;
      if (digit2 & (1<<(7-i))) stream[i+9]=1;
      if (digit3 & (1<<(7-i))) stream[i+17]=1;
    }

  for ( i=0; i<36; i++ ) {
      // clock low
      PORTE &= ~(1<<3);
      // data enable low
      PORTH &= ~(1<<4);
      _delay_us (1);
      // data
      if (stream[i]==1)
        PORTH |= (1<<3);
      else
        PORTH &=~(1<<3);
      _delay_us (1);
      // clock high - Transmission finished
      PORTE |= (1<<3);
      _delay_us (1);
      // data enable high - ready for next cycle
      PORTH |= (1<<4);
    }
}
