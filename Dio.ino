#include <avr/io.h>

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