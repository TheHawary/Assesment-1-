# Assesment-1

## Code:
### App.ino

```
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
```


### adc.h

```#ifndef __ADC__
#define __ADC__

void Adc_Init(void);

unsigned short Adc_ReadChannel(unsigned char channel);

#endif
```

### adc.ino

```#include "Adc.h"

void Adc_Init(void)
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);
 
    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

unsigned short Adc_ReadChannel(unsigned char ch)
{

  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
 
  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);

}
```

### uart.h

```#ifndef __UART__
#define __UART__

void Uart_Init(void);

void Uart_SetBaudRate(unsigned short BuadRate);

void Uart_SendChar(unsigned char DataByte);

unsigned char Uart_IsDataAvailable(void);

unsigned char Uart_ReadData();

void Uart_SendString(unsigned char DataString[]);

#endif
```


### uart.ino

```#if !defined(__AVR_ATmega328P__)
#include <avr/iom328p.h>
#endif

#include "Uart.h"

#define _BV(bit) (1 << (bit))
#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))
#define loop_until_bit_is_set(sfr, bit) do { } while (bit_is_clear(sfr, bit))
#define loop_until_bit_is_clear(sfr, bit) do { } while (bit_is_set(sfr, bit))


void Uart_Init(void) {

    Uart_SetBaudRate(9600);

    /* Enable USART transmitter/receiver */
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    /* 8 data bits, 1 stop bit */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void Uart_SetBaudRate(unsigned short BuadRate)
{
    unsigned short UBBR = ( (F_CPU / 16) /  BuadRate ) - 1;
    UBRR0L = (UBBR & 0xFF);
    UBRR0H = (( UBBR >> 8 ) & 0xFF);
}


void Uart_SendChar(unsigned char DataByte)
{
    // Wait until Write buffer is empty
    while ( ! (UCSR0A & ( 1 << UDRE0)) );
    UDR0 = DataByte;
}

unsigned char Uart_ReadData(void) {
    // Wait until data is received
    while ( ! (UCSR0A & ( 1 << RXC0)) );
    return UDR0;
}

void Uart_SendString(const char DataString[], unsigned char Size){
    int i;
    for (i=0; DataString[i]; i++)
    {
        Uart_SendChar(DataString[i]);
    }
}
```


### Dio.h

```#ifndef __Dio__
#define __Dio__

void Dio_SetPinDirection(unsigned char DDRX,unsigned char Pin,unsigned char Direction)

void Dio_SetPortDirection(unsigned char PORTX,unsigned char Pin,unsigned char Direction)

unsigned char Dio_GetPinState(unsigned char Port,unsigned char pin)


#endif
```

### Dio.ino

```#include <avr/io.h>

#define SetBit(Reg,Pin) (Reg|=(1<<Pin))
#define ClearBit(Reg,Pin) (Reg&=~(1<<Pin))
#define GetBit(Reg,Pin) (Reg&(1<<Pin))

void Dio_SetPinDirection(unsigned char DDRX,unsigned char Pin,unsigned char Direction){
  switch(DDRX)
  {
     case'B':
     if(Direction==1)
     {
       SetBit(DDRB,Pin);
     }
     else{
       ClearBit(DDRB,Pin);
     }
      break;
        case'C':
     if(Direction==1)
     {
       SetBit(DDRC,Pin);
     }
     else{
       ClearBit(DDRC,Pin);
     }
      break;
      case'D':
     if(Direction==1)
     {
       SetBit(DDRD,Pin);
     }
     else{
       ClearBit(DDRD,Pin);
     }
      break;
  }
}

void Dio_SetPortDirection(unsigned char PORTX,unsigned char Pin,unsigned char Direction){
  switch(PORTX)
  {
     case'B':
     if(Direction==1)
     {
       SetBit(PORTB,Pin);
     }
     else{
       ClearBit(PORTB,Pin);
     }
      break;
        case'C':
     if(Direction==1)
     {
       SetBit(PORTC,Pin);
     }
     else{
       ClearBit(PORTC,Pin);
     }
      break;
      case'D':
     if(Direction==1)
     {
       SetBit(PORTD,Pin);
     }
     else{
       ClearBit(PORTD,Pin);
     }
      break;
  }
}
unsigned char Dio_GetPinState(unsigned char Port,unsigned char pin){
  int bitval;
  switch(Port)
  {
     case 'B':
     bitval = GetBit(PORTB,pin);
     break;
     case 'C':
     bitval = GetBit(PORTC,pin);
     break;
     case 'D':
     bitval = GetBit(PORTD,pin);
     break;
  }
}
```


## Further Improvements
- The Dio library could be implented to further improve the simplicity of the code.
- The response time could be improved.
- A switch could be used instead of a button to operate the application easier




