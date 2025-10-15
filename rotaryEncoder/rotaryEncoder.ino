#include <util/delay.h>

enum laundryStep {WASHING = 0, WATERTEMP, RINSECOUNT, DEHYDRATIONTIME, RUN};
enum laundryStep state;
volatile uint8_t encoderValue = 0;
volatile uint8_t lcd_on = 0;
volatile uint8_t ackSTM;
unsigned long buttonPressStart = 0;
volatile bool buttonPressed = false;
volatile bool dontCount = false;
uint8_t laundryData[4];
uint8_t sendingStateData[5] = {3 /* led_on */, 4 /* WASHING select */, 5/* WATERTEMP select */, 6/*RINSECOUNT select */, 7/* DEHYDRATION select*/};

void SPI_init()
{
  DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2) | (1 << PB1);
  DDRB &= ~(1 << PB4);
  PORTB |= (1 << PB2) | (1 << PB1);
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_sendDataToSTM(uint8_t data)
{
  PORTB &= ~(1 << PB2);
  SPDR = data;
  while (!(SPSR & (1 << SPIF)));
  ackSTM = SPDR;
  PORTB |= (1 << PB2);
  _delay_ms(10);
}

void SPI_sendDataToAr(uint8_t data)
{
  PORTB &= ~(1 << PB1);
  SPDR = data;
  while (!(SPSR & (1 << SPIF)));
  PORTB |= (1 << PB1);
  _delay_ms(10);
}

ISR(INT0_vect)
{
  dontCount = false;
  if (!(PIND & (1 << PD2)) & (lcd_on == 1))
  {
    if (state == WASHING)
    {
      encoderValue = (encoderValue + 1) % 2;
      SPI_sendDataToSTM(encoderValue);
    }
    else if ((state > WASHING) & (state < RUN))
    {
      encoderValue = (encoderValue + 1) % 3;
      SPI_sendDataToSTM(encoderValue);
    }
  }
}

ISR(INT1_vect)
{
  dontCount = false;
  if (!(PIND & (1 << PD3)) & (lcd_on == 1))
  {
    if (state == WASHING)
    {
      encoderValue = (encoderValue - 1 + 2) % 2;
      SPI_sendDataToSTM(encoderValue);
    }
    else if ((state > WASHING) & (state < RUN))
    {
      encoderValue = (encoderValue - 1 + 3) % 3;
      SPI_sendDataToSTM(encoderValue);
    }
  }
}

void setup(void) 
{
  SPI_init();
  DDRD &= ~(1 << PD2) & ~(1 << PD3) & ~(1 << PD4);
  PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4);
  EICRA |= (1 << ISC11);
  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC01);
  EIMSK |= (1 << INT0);
  sei();
}
    
    
void loop(void) 
{
  if (lcd_on == 0)
  {
    if (!(PIND & (1 << PD4)))
    {
      _delay_ms(3000);
      if (!(PIND & (1 << PD4)))
      {
        lcd_on = 1;
        dontCount = true;
        SPI_sendDataToSTM(sendingStateData[0]); // led_on
      }
    }
  }
  else
  {
    if (!(PIND & (1 << PD4)) && !dontCount)
    {
      buttonPressStart = millis();
      buttonPressed = true;
    }
    if ((PIND & (1 << PD4)) && buttonPressed)
    {
      unsigned long pressDuration = millis() - buttonPressStart;
      if (pressDuration <= 500)
      {
        switch (state)
        {
          case WASHING :
            laundryData[0] = encoderValue;
            SPI_sendDataToSTM(sendingStateData[1]); // WASHING select
            _delay_ms(10);         
            if (ackSTM == 0x20)
            {
              SPI_sendDataToAr(encoderValue);
              state = WATERTEMP;     
            }
            break;
          case WATERTEMP :
            laundryData[1] = encoderValue;  
            SPI_sendDataToSTM(sendingStateData[2]); // WATERTEMP select
            _delay_ms(10);            
            if (ackSTM == 0x20)
            {
              SPI_sendDataToAr(encoderValue);
              state = RINSECOUNT;                   
            }
            break;
          case RINSECOUNT :
            laundryData[2] = encoderValue;
            SPI_sendDataToSTM(sendingStateData[3]); // RINSECOUNT select
            _delay_ms(10);
            if (ackSTM == 0x20)
            {
              SPI_sendDataToAr(encoderValue);
              state = DEHYDRATIONTIME;         
            }
            break;
          case DEHYDRATIONTIME :
            laundryData[3] = encoderValue;
            SPI_sendDataToAr(encoderValue);
            SPI_sendDataToSTM(sendingStateData[4]); // DEHYDRATION select
            _delay_ms(10); 
            if (ackSTM == 0x20)
            {
              SPI_sendDataToAr(encoderValue);
              state = RUN;             
            }
            break;
          case RUN :
            if (ackSTM == 0x20)
              state = WASHING;
            break;
        }
      }
      buttonPressed = false;
    }
  }
}
