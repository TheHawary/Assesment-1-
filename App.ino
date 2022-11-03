#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "adc.h"

int main(void){
  init();
  unsigned char buffer[4] ;                          // Creating an array to be used later on in the itoa function
  while(1)
  { 
    int ldr_value = Adc_ReadChannel(0);              // Reading the LDR value

    // Light detecting

    if(ldr_value < 50){                              // If there is no light on the LDR (more sensitve to light)
      PORTD = PORTD | (1 << 4);                      // LED stays turned on 
      
    }
    else{
      PORTD &= ~(1 << 4);                            // LED turns off
    }

    // Using the button to control the light detection sensitivity
    
    if((PINB & (1 << 3))!= 0)                        // if button not pressed
    {
      PORTB = PORTB & ~(1 << 5);                     // Switch OFF LED to indicate that the button is not pressed

    }  
    else{
      if(ldr_value < 300){                           // If there is no light on the LDR (less sesitive to light)
        PORTD = PORTD | (1 << 4);                    // LED stays turned on 
        }
      else{
      PORTD &= ~(1 << 4);                            // LED turns off
      }

      
      PORTB = PORTB | (1 << 5);                      // Switch ON LED to indicate the button is pressed

      // Sending message to indicate the button is pressed
      
      Uart_SendChar('B' );
      Uart_SendChar('u');
      Uart_SendChar('t');
      Uart_SendChar('t');
      Uart_SendChar('o');
      Uart_SendChar('n');
      Uart_SendChar(' ');
      Uart_SendChar('i');
      Uart_SendChar('s');
      Uart_SendChar(' ');
      Uart_SendChar('p');
      Uart_SendChar('r');
      Uart_SendChar('e');
      Uart_SendChar('s');
      Uart_SendChar('s');
      Uart_SendChar('e');
      Uart_SendChar('d');      
      Uart_SendChar('\n');
      _delay_ms(200);
      
    }

       
      itoa(ldr_value, buffer, 10);                   //itoa function to covert numbers from the ldr_value to a string

      // Representing and showing the light intensity in numbers
      
      Uart_SendChar('L');
      Uart_SendChar('i');
      Uart_SendChar('g');
      Uart_SendChar('h');
      Uart_SendChar('t');
      Uart_SendChar(' ');
      Uart_SendChar('i');
      Uart_SendChar('n');
      Uart_SendChar('t');
      Uart_SendChar('e');
      Uart_SendChar('n');
      Uart_SendChar('s');
      Uart_SendChar('i');
      Uart_SendChar('t');
      Uart_SendChar('y');
      Uart_SendChar(':');
      Uart_SendChar('\t');
      Uart_SendString(buffer,3);
      Uart_SendChar('\n');
      _delay_ms(500);
    
  }

}

void init(){
  Uart_Init();
  Adc_Init();
  DDRC = 0x00;                // Make port C input
  DDRD = DDRD | (1 << 4);     // Make PD4 output

  
  DDRB = DDRB | (1 << 5);     // Make PB5 output
  DDRB = DDRB & ~(1 << 3);    // Make PB3 input
  PORTB = PORTB | (1 << 3);   // Activate pull up resistance
  
}
