#define __SFR_OFFSET 0
#include "avr/io.h"

.global main
main: 
  ldi   r16, 0xFF  
  out   DDRB, r16
  
  ; FastPWM            1<<WGM12
  ; Phase-Correct PWM  0<<WGM12
  
  ;  8-bit counter     0<<WGM11 | 1<<WGM10
  ;  9-bit counter     1<<WGM11 | 0<<WGM10
  ; 10-bit counter     1<<WGM11 | 1<<WGM10
  
  ; inverted mode      1<<COM1C1 | 1<<COM1C0
  ; non-inverted mode  1<<COM1C1 | 0<<COM1C0
  
  ; prescaler:         CS12 | CS11 | CS10 
  ;                    Table 17-6 ... Seite 161       
  
  ldi   r16, 0<<COM1A1 | 0<<COM1A0 | 0<<COM1B1 | 1<<COM1B0 | 1<<COM1C1 | 0<<COM1C0 | 1<<WGM11 | 1<<WGM10
  sts   TCCR1A, r16
  
  ldi   r16, 0<<ICNC1  | 0<<ICES1  |   0<<5    |  0<<WGM13 |  0<<WGM12 | 0<<CS12   | 0<<CS11  | 1<<CS10 
  sts   TCCR1B, r16
  
  ldi   r17, 100
  ldi   r16, 0
  
  sts   OCR1CH, r16
  sts   OCR1CL, r17  

loop:
  rjmp  loop
  
