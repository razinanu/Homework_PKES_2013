/*
 * counter_func.S
 *
 *  Created on: Sep 20, 2013
 *      Author: zug
 */

#include <avr/io.h>
#include "def.h"

.global counter_func

.extern counter_trigger
.extern counter_dir
.extern counter_value

counter_func:	lds r24, counter_dir
				lds r25, counter_trigger   ;load current trigger value
				lds r26, counter_value


               sbic _SFR_IO_ADDR(PINF), PINF4 ; set if button down
			   ldi r24,1


             ; Einlesen der Buttons
        	   sbic _SFR_IO_ADDR(PING), PING5 ; set if button 1
			   ldi r24,0

                cpi  r25,5
                brne trigger
                ldi r25,0
                cpi r24,0
                breq decrement
                jmp  increment

trigger:       inc r25
               jmp end



decrement:      dec  r26
                jmp  end

increment:      inc r26


end:
                sts counter_dir, r24
				sts counter_trigger, r25   ;save counter_trigger
				sts counter_value, r26     ;
