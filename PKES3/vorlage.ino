#include <math.h>
#include <util/delay.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "Flydurino.h"

void* operator new(size_t s, void* ptr) {
	return ptr;
}
void setup();
void loop();
int8_t checkButtons();
void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z);
uint8_t linearizeDistance(uint16_t distanceRaw);
void displayDistance(int8_t dist);
uint16_t readADC(int8_t channel);
void writetoDisplay(char digit1, char digit2, char digit3);
uint8_t displayMask(char val);

const int NUM_READS = 10;
float sortedValues[NUM_READS];
int buffer = 0;
//4 Aufgabe
float differLeft = 0;
int fisrtTick = 0;
int turnValue = 350;
bool turnBack = false;
bool twoTurned = false;
bool turned = false;
bool startTick = true;
volatile int ticksLeft = 0;
volatile int ticksRight = 0;
double distanceLeft = 0;
double distanceRight = 0;

char flydurinoPtr[sizeof(Flydurino)];
// aktuelle Beschleunigungswerte, Kompassmessungen
int16_t acc_x, acc_y, acc_z;
int16_t ori_x, ori_y, ori_z;
int16_t rot_x, rot_y, rot_z;
float current_rot_deg, sum_rot;
// Wasserwaage oder Distanzmessung
int8_t modus;
// Kanal des ADC Wandlers
// -------------------------------------------------------------
int8_t channelLeft = 2; // korrekte Werte bestimmen !
int8_t channelRight = 3;
// -------------------------------------------------------------
// Laufzeit des Programms
unsigned long currentTime;
unsigned long lastTime;
unsigned long deltaTime;

int countZeros = 0;

//cm
const double kc = (3.14159265 * 5) / 120;

//float targetValue;

enum display{
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Z9, MINUS_1, MINUS, SPACE
};

enum motorMode {
	MOTOR_FORWARD,
	MOTOR_TURN_LEFT,
	MOTOR_TURN_RIGHT,
	MOTOR_STOP,
	MOTOR_ROTATE_LEFT,
    	MOTOR_ROTATE_RIGHT,
    	MOTOR_BACKWARD
};
enum speed
{
    SPEED_HIGH, SPEED_MEDIUM, SPEED_SLOW,SPEED_VERY_SLOW
};

// Install the interrupt routine.
ISR(INT4_vect) {
	ticksRight++;
}

ISR(PCINT0_vect) {
	ticksLeft++;
}

// the setup routine runs once when you press reset:
void setup()
{
	// initialize serial communication
	Serial.begin(9600);

	Serial.println("----------------------------------");
	Serial.println("PKES Wintersemester 2013/14");
	Serial.println("Vorlage 3. Aufgabe ");
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
	DDRB |= (1 << DDA7);
	// disable leds
	PORTB &= ~(1 << 7);
	// -------------------------------------------------------------
	// Serial bus lines
	// -------------------------------------------------------------
	// Pin 5 = PORT E 3 = clock
	DDRE |= (1 << DDE3);
	// Pin 6 = PORT H 3 = data
	DDRH |= (1 << DDH3);
	// Pin 7 = PORT H 4 = enable
	DDRH |= (1 << DDH4);
	// -------------------------------------------------------------
	// Button configuration
	// -------------------------------------------------------------
	// not necessary but for completion
	// S1 as input
	DDRF &= ~(1 << DDF4);
	// S2 as input
	DDRG &= ~(1 << DDG5);

	// Configuration of ADC
	// -----------------------------------------------------
	analogReference(INTERNAL2V56);
	// -----------------------------------------------------

	// Configure buttons
	// -----------------------------------------------------
	digitalRead(4); // S1

	//  Configure PWM
	//  Digital 11 | OC1A:
	/*  OC1A, Output Compare Match A output: The PB5 pin can serve as an external output for the
	 Timer/Counter1 Output Compare A. The pin has to be configured as an output (DDB5 set (one))
	 to serve this function. The OC1A pin is also the output pin for the PWM mode timer function.
	 */
	//  Digital  3 | OC3C:
	/*  OC3C, Output Compare Match C output: The PE5 pin can serve as an External output for the
	 d      Timer/Counter3 Output Compare C. The pin has to be configured as an output (DDE5 set “one”)
	 to serve this function. The OC3C pin is also the output pin for the PWM mode timer function.
	 */
	/*  TCNT1(TCCR1x)/OCR1A/OC1A einsstellen und
	 TCNT3(TCCR3x)/OCR3C/OC3C einstellen / Prescaler=256 (100) CS12:0
	 Waveform Generation Mode = 5 (0101) WGM3:0
	 COMnA1/COMnB1/COMnC1 = 1 / COMnA0/COMnB0/COMnC0=0*/
	// -----------------------------------------------------
    pinMode(11,OUTPUT);
    TCCR1A = 0 << COM1A1 | 0 << COM1A0 | 0 << COM1B1 | 0 << COM1B0 | 0 < COM1C1 | 0 << COM1C0 | 0 << WGM11 | 1 << WGM10;
    TCCR1B = 0 << ICNC1 | 0 << ICES1 | 0 << 5| 0<<WGM13 | 1<<WGM12 | 0<<CS12 | 0<<CS11 | 1<<CS10;OCR1A=150;

    pinMode (3,OUTPUT);
    TCCR3A = 0 << COM3A1 | 0 << COM3A0 | 0 << COM3B1 | 0 << COM3B0 | 0 << COM3C1 | 0 << COM3C0 | 0 << WGM31 | 1 << WGM30;
    TCCR3B = 0 << ICNC3 | 0 << ICES3 | 0 << 5| 0<<WGM33 | 1<<WGM32 | 0<<CS32 | 0<<CS31 | 1<<CS30;OCR3C=150;

    pinMode (13	,OUTPUT);
	digitalWrite(13, HIGH);

	pinMode(12, OUTPUT);
	digitalWrite(12, HIGH);
	/**/
	// -----------------------------------------------------
	// Configure Flydurino
	new (flydurinoPtr) Flydurino;
	// -----------------------------------------------------
	// Tobi: This sets the Digital Low Pass Filter on the MPU6050
	// mode 6 is the strongest Filter and should make everything smoother
	((Flydurino*) flydurinoPtr)->setDLPFMode(6);
	// -----------------------------------------------------
	// turn on interrupts
//    pinMode(2,INPUT);
//    pinMode(10,INPUT);
	PCICR = (1 << PCIE0);	// any change on PCINT7:0 will cause interrupt
	PCMSK0 = (1 << PCINT4);	// enable pin change interrupt 4

	EICRB = (1 << ISC40);	// generate interrupt on any logical change on INT4

	EIMSK = (1 << INT4);	// enable external interrupt 4

	EICRB = (1 << ISC40);
	EIMSK = (1 << INT4);
	// enable global interrupts
	sei();
	
//      EICRA = (1 << ISC01 | 1<<ISC00);
// -----------------------------------------------------
	lastTime = millis();
	modus = 0;
	sum_rot = 0;
}

/// displays degrees divided by 10
void displayDegrees() {
	int displaySumRot = (int) sum_rot;
	uint8_t sign;
	uint8_t first;
	uint8_t second;
	if (displaySumRot >= 0)
		sign = displayMask(' ');
	else
		sign = displayMask('-');
	displaySumRot = abs(displaySumRot);
	first = 48 + (int) displaySumRot / 100;
	second = 48 + (int) ((displaySumRot % 100) / 10);
	writetoDisplay(sign, displayMask((char) first), displayMask((char) second));
}
void setSpeed(int speedMode)

{

	switch (speedMode) {

	case SPEED_HIGH:
		OCR1A = 90;
		OCR3C = 90 + 2 * differLeft;
		break;
	case SPEED_MEDIUM:
		OCR1A = 120;
		OCR3C = 120 + 2 * differLeft;
		break;
	case SPEED_SLOW:
		OCR1A = 150;
		OCR3C = 150 + 2 * differLeft;
		break;
	case SPEED_VERY_SLOW:
		OCR1A = 160;
		OCR3C = 160 + 2 * differLeft;
		break;
	}
}

void setMotor(int motorMode) {
	switch (motorMode) {
	case MOTOR_FORWARD:
		TCCR1A |= (1 << COM1A1 | 1 << COM1A0);
		TCCR3A |= (1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, HIGH);
		digitalWrite(12, HIGH);
		break;
	case MOTOR_TURN_LEFT:
		Serial.print("links");
		TCCR1A |= (1 << COM1A1 | 1 << COM1A0);
		TCCR3A &= ~(1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, HIGH);
		digitalWrite(12, HIGH);
		break;
	case MOTOR_TURN_RIGHT:
		Serial.print("rechts");
		TCCR1A &= ~(1 << COM1A1 | 1 << COM1A0);
		TCCR3A |= (1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, HIGH);
		digitalWrite(12, HIGH);
		break;
	case MOTOR_ROTATE_LEFT:
		TCCR1A |= (1 << COM1A1 | 1 << COM1A0);
		TCCR3A |= (1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, HIGH);
		digitalWrite(12, LOW);
		break;
	case MOTOR_ROTATE_RIGHT:
		TCCR1A |= (1 << COM1A1 | 1 << COM1A0);
		TCCR3A |= (1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, LOW);
		digitalWrite(12, HIGH);
		break;
    case MOTOR_BACKWARD:
        TCCR1A |= (1 << COM1A1 | 1 << COM1A0);
        TCCR3A |= (1 << COM3C1 | 1 << COM3C0);
        digitalWrite(13, LOW);
        digitalWrite(12, LOW);
        break;
	default:
		TCCR1A &= ~(1 << COM1A1 | 1 << COM1A0);
		TCCR3A &= ~(1 << COM3C1 | 1 << COM3C0);
		digitalWrite(13, HIGH);
		digitalWrite(12, HIGH);
		break;
	}
}

void calculateGyro() {
	currentTime = millis();
	deltaTime = currentTime - lastTime;
	lastTime = currentTime;
	// Receive acceleromation values
	((Flydurino*) (flydurinoPtr))->getAcceleration(&acc_x, &acc_y, &acc_z);

	// Get compass data
	((Flydurino*) (flydurinoPtr))->getOrientation(&ori_x, &ori_y, &ori_z);
	// Get gyro data
	((Flydurino*) (flydurinoPtr))->getRotationalSpeed(&rot_x, &rot_y, &rot_z);
	/*Empirically determined Offset:
	 Offset 81 was determined with ~1200 Samples and MPU6050 Digital Low Pass Mode 6*/
	int16_t rot_z_offset = 81;
	// Substract Offset
	rot_z -= rot_z_offset;
	/**/
	/*FS_SEL | Full Scale Range   | LSB Sensitivity
	 -------+--------------------+----------------
	 0      | +/- 250 degrees/s  | 131 LSB/deg/s
	 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
	 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
	 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s */
	uint8_t fs_sel = ((Flydurino*) (flydurinoPtr))->getFullScaleGyroRange();
	switch (fs_sel) {
	case 0:
		rot_z = (int16_t) ((rot_z / 131));
		break;
	case 1:
		rot_z = (int16_t) ((rot_z / 65.5));
		break;
	case 2:
		rot_z = (int16_t) ((rot_z / 32.8));
		break;
	case 3:
		rot_z = (int16_t) ((rot_z / 16.4));
		break;
	}
	// Integration
	double secs = (double) (deltaTime) / 1000;
	current_rot_deg = rot_z * (secs);
	sum_rot = sum_rot + current_rot_deg;
}

void gyroTask() {
	// Receive acceleromation values

//	Serial.print("zeros: ");
//	Serial.print(countZeros);
//	Serial.print("\t");

	if (current_rot_deg == 0) {
		countZeros++;
	} else {
		countZeros = 0;
	}

	if (countZeros == 20) {
		countZeros = 0;
		turnBack = true;
	}

	if (turnBack) {
		if (abs(sum_rot) > 10) {
			if (sum_rot > 0) {

				if (sum_rot > 50) {
					setSpeed(SPEED_HIGH);
					setMotor(MOTOR_ROTATE_RIGHT);
				} else if (sum_rot > 30) {
					setSpeed(SPEED_MEDIUM);
					setMotor(MOTOR_ROTATE_RIGHT);
				} else if (sum_rot > 20) {
					setSpeed(SPEED_SLOW);
					setMotor(MOTOR_ROTATE_RIGHT);
				}

			} else {
				if (abs(sum_rot) > 50) {
					setSpeed(SPEED_HIGH);
					setMotor(MOTOR_ROTATE_LEFT);
				} else if (abs(sum_rot) > 30) {
					setSpeed(SPEED_MEDIUM);
					setMotor(MOTOR_ROTATE_LEFT);
				} else if (abs(sum_rot) > 20) {
					setSpeed(SPEED_SLOW);
					setMotor(MOTOR_ROTATE_LEFT);
				}
			}
		} else {
			setMotor(MOTOR_STOP);
			turnBack = false;
		}
	}

	//display degrees divided by ten
	displayDegrees();

	/*
	 Serial.print("fs_sel: ");Serial.print(fs_sel);
	 Serial.print(" sum_rot: ");Serial.print(sum_rot); Serial.print("\t");
	 //	 Serial.print(rot_x); Serial.print("\t");
	 //	 Serial.print(rot_y); Serial.print("\t");
	 //	 Serial.print("R Z: ");Serial.print(rot_z);
	 //	 Serial.print("\t");
	 Serial.print("C Z: ");Serial.print(current_rot_deg);Serial.print("\t");
	 Serial.print("secs: ");Serial.print(secs);Serial.print("\t");
	 Serial.print("dT: ");Serial.print(deltaTime);//Serial.print();
	 /**/
	Serial.print("\r\n");
	delay(200);
	return;
}

void motorTask() {
	setSpeed(SPEED_HIGH);
	setMotor(MOTOR_FORWARD);

	uint8_t distance_left, distance_right;
	distance_right = linearizeDistance(readADC(channelRight));
	distance_left = linearizeDistance(readADC(channelLeft));

	if (distance_right < 15) {
		if (distance_right < 10) {
			setMotor(MOTOR_ROTATE_LEFT);
		}

		else {
			setSpeed(SPEED_SLOW);
			setMotor(MOTOR_TURN_LEFT);
		}

	}
	if (distance_left < 15) {

		if (distance_left < 10) {
			setMotor(MOTOR_ROTATE_RIGHT);
		}

		else {
			setSpeed(SPEED_SLOW);
			setMotor(MOTOR_TURN_RIGHT);
		}

	}

	delay(50);
}

void turnTask() {
//
//    if(!turned){
//    	unsigned long currentTime=millis();
//        setSpeed(SPEED_SLOW);
//        setMotor(MOTOR_FORWARD);
//        // calculate moved distance and integrate
//        distanceLeft += kc * (double) ticksLeft * 0.5;
//        unsigned long lastTime=millis();
//        unsigned long differTime=lastTime-currentTime;
//        speedLeft=distanceLeft/differTime;
//        Serial.print("SPEED Left: ");
//         Serial.print(speedLeft);
//        distanceRight += kc * (double) ticksRight * 0.5;
//        speedRight=distanceRight/currentTime;
//        if(abs(distanceLeft-distanceRight)<5){
//        	if(distanceRight>distanceRight){
//        		differLeft=abs(speedLeft-speedRight);
//        	}
//        	else{
//        		differRight=abs(speedLeft-speedRight);
//        	}
//        }
//        displayDegrees();
//        // turn after 50
//        if (!turned && ((distanceLeft + distanceRight) / 2) > 50.0)
//        {
//           if(startdegree){
//                //           startValue=sum_rot;
//                targetValue = sum_rot+180;
//                startdegree=false;
//                setSpeed(SPEED_VERY_SLOW);
//                setMotor(MOTOR_ROTATE_LEFT);
//            }
//
//            //       if( abs(startValue-sum_rot)>90 && abs(startValue-sum_rot)<180){
//            //           setSpeed(SPEED_SLOW);
//            //           setMotor(MOTOR_ROTATE_LEFT);
//            //       }
//            //        if(abs(startValue-sum_rot)<90){
//            //            setSpeed(SPEED_VERY_SLOW);
//            //            setMotor(MOTOR_ROTATE_LEFT);
//            //        }
//            //        else{
//            //            startdegree=true;
//            //            turned=true;
//            //        }
//
//            //        turned = true;
//            //        targetSumRot = sum_rot;
//            while (!turned)
//            {
//                calculateGyro();
//                displayDegrees();
//                float mySum = targetValue-sum_rot;
//                if(abs(mySum)>2.5){
//                    if (mySum > 0)
//                    {
//                        setSpeed(SPEED_SLOW);
//                        setMotor(MOTOR_ROTATE_LEFT);
//                    }
//                    else
//                    {
//                        setSpeed(SPEED_SLOW);
//                        setMotor(MOTOR_ROTATE_RIGHT);
//                    }
//                }else
//                {
//                    turned = true;
//                    setMotor(MOTOR_STOP);
//                }
//
//                delay(200);
//                //		}
//                //        setSpeed(SPEED_SLOW);
//                //        setMotor(MOTOR_FORWARD);
//                //        return;
//            }
//        }
//    }else{
//
//    	setSpeed(SPEED_SLOW);
//
//        setMotor(MOTOR_FORWARD);
//
//        // calculate moved distance and integrate
//        distanceLeft += kc * (double) ticksLeft * 0.5;
//        distanceRight += kc * (double) ticksRight * 0.5;
//
//        displayDegrees();
//
//        if (turned && ((distanceLeft + distanceRight) / 2) > 100.0)
//        {
//            setMotor(MOTOR_STOP);
//        }
//    }
//    // drive back
//    // stop
//    Serial.print("LEFT: ");
//    Serial.print(distanceLeft);
//    Serial.print(" cm\t");
//    Serial.print("SPEED Left: ");
//    Serial.print(speedLeft);
//    Serial.print(" \t");
//    Serial.print("SPEED Right: ");
//    Serial.print(speedRight);
//    Serial.print(" \t");
//    Serial.print("RIGHT: ");
//    Serial.print(distanceRight);
//    Serial.print(" cm\r\n");

	setSpeed(SPEED_MEDIUM);
	setMotor(MOTOR_FORWARD);

	distanceLeft += kc * (double) ticksLeft * 0.5;
	distanceRight += kc * (double) ticksRight * 0.5;

		differLeft = ticksLeft - ticksRight;

	// turn after 50
	if (!turned && ((distanceLeft + distanceRight) / 2) > 50.0) {

		if (startTick) {
			fisrtTick = ticksLeft;
			startTick = false;
		}

		while (abs(startTick-ticksLeft) < turnValue) {

			setSpeed(SPEED_MEDIUM);
			setMotor(MOTOR_ROTATE_LEFT);

		}

		turned = true;
		setSpeed(SPEED_MEDIUM);
		setMotor(MOTOR_FORWARD);
	}
	if (turned && !twoTurned && ((distanceLeft + distanceRight) / 2) > 100.0) {

		while (abs(startTick-ticksLeft) < turnValue) {

			setSpeed(SPEED_MEDIUM);
			setMotor(MOTOR_ROTATE_LEFT);

		}
		twoTurned = true;
		setMotor(MOTOR_STOP);

	}
	if (turned && twoTurned) {
		setMotor(MOTOR_STOP);
	}
	ticksLeft = 0;
	ticksRight = 0;
	delay(200);
}

void resetAll() {
	setMotor(MOTOR_STOP);
	modus = 0;
	sum_rot = 0;
	ticksLeft = 0;
	ticksRight = 0;

}



void swarmTask()
{
    uint8_t distanceToWall, distanceForward;
    distanceForward = linearizeDistance(readADC(channelRight));
    distanceToWall = linearizeDistance(readADC(channelLeft));
    displayDistance(distanceToWall);

//    Serial.print("FWD: ");
//    Serial.print(distanceForward);
//    Serial.print("\t");
//    Serial.print("WALL: ");
//    Serial.print(distanceToWall);
//    Serial.print("\r\n");

//    analogWrite(11,255);
//    analogWrite(3,255);
    if (distanceForward >= 10)
    {
        setMotor(MOTOR_FORWARD);
        if (distanceToWall < 17)
        {
            setMotor(MOTOR_TURN_LEFT);
        }
    }
    if (distanceForward < 10)
    {
        setMotor(MOTOR_BACKWARD);
    }



    delay(50);
}

void loop()
{
    //calculateGyro();
	// default state - avoids crash situations due to suddenly starting
	// PWM modus
	if (modus == 0) {
		writetoDisplay(0b10011111, 0b11111101, 0b10110111);

		while (modus == 0) {
			modus = checkButtons();
		}
	}
	// Gyro task
	if (modus == 1) {
        swarmTask();
	}
	// Driving without any collision
	if (modus == 2) {
        swarmTask();
	}

	modus = checkButtons();
}

int8_t checkButtons() {
	int8_t modus_new = modus;
	// Abfrage der Buttons und Moduswechsel
	// -----------------------------------------------------

	if (digitalRead(4)) {

		modus_new = 1;
	}
	if (analogRead(4) > 800) {
		modus_new = 2;
	}
	if (modus != modus_new) {
		setMotor(MOTOR_STOP);
	}

	return modus_new;
}

void displaySpiritLevel(int16_t acc_x, int16_t acc_y, int16_t acc_z) {

	//   3 cases for roll and pitch
	// -15 Grad <= alpha,
	// -15 Grad <= alpha  <= 15 Grad
	//  15 Grad <= alpha
	// -----------------------------------------------------

	// -----------------------------------------------------
}

uint8_t linearizeDistance(uint16_t distance_raw) {
	double distance_cm = 0;
	distance_cm = 2 * ((3500 / (double) (distance_raw + 4)) - 1);

	// Transformation der Spannungsbezogenen Distanzwerte in
	// eine Entfernung in cm
	// -----------------------------------------------------

	// -----------------------------------------------------
	return (int8_t) ceil(distance_cm);
}

void displayDistance(int8_t dist)
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

uint16_t readADC(int8_t channel) {
	uint16_t distance_raw = 0xFFFF;
	// möglicherweise mehrmaliges Lesen des ADC Kanals
	// Mittelwertbildung
	// -----------------------------------------------------

	int sum = 0;
	for (int i = 0; i < NUM_READS; i++) {
		sortedValues[i] = analogRead(channel);

	}

	for (int i = 0; i < NUM_READS; i++) {
		sum += sortedValues[i];
	}

	distance_raw = sum / NUM_READS;

	return distance_raw;
}

void writetoDisplay(char digit1, char digit2, char digit3) {

	char stream[36];
	stream[0] = 1;
	int i;
	for (i = 1; i < 36; i++) {
		stream[i] = 0;
	}

	for (i = 0; i < 8; i++) {
		if (digit1 & (1 << (7 - i)))
			stream[i + 1] = 1;
		if (digit2 & (1 << (7 - i)))
			stream[i + 9] = 1;
		if (digit3 & (1 << (7 - i)))
			stream[i + 17] = 1;
	}

	for (i = 0; i < 36; i++) {
		// clock low
		PORTE &= ~(1 << 3);
		// data enable low
		PORTH &= ~(1 << 4);
		_delay_us(1);
		// data
		if (stream[i] == 1)
			PORTH |= (1 << 3);
		else
			PORTH &= ~(1 << 3);
		_delay_us(1);
		// clock high - Transmission finished
		PORTE |= (1 << 3);
		_delay_us(1);
		// data enable high - ready for next cycle
		PORTH |= (1 << 4);
	}
}

uint8_t displayMask(char val) {
	switch (val) {
	case ' ':
		return 0b00000000;
	case '0':
		return 0b11111100;

	case '1':
		return 0b01100000;
	case '2':
		return 0b11011010;
	case '3':
		return 0b11110010;
	case '4':
		return 0b01100110;
	case '5':
		return 0b10110110;
	case '6':
		return 0b10111110;
	case '7':
		return 0b11100000;
	case '8':
		return 0b11111110;
	case '9':
		return 0b11110110;

	case 'a':
	case 'A':
		return 0b11101110;
	case 'b':
	case 'B':
		return 0b00111110;
	case 'c':
		return 0b00011010;
	case 'C':
		return 0b10011100;
	case 'd':
	case 'D':
		return 0b01111010;
	case 'e':
	case 'E':
		return 0b10011110;
	case 'f':
	case 'F':
		return 0b10001110;

	case '-':
		return 0b00000010;

	default:
		return 0b00000001;
	}
}
