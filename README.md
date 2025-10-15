<p align="center">
  <h1 align="center">로타리 엔코더 기반 스마트 세탁기 제어 시스템✨</h1>
</p>

## Index ⭐
- [1. 프로젝트 목표](#1-프로젝트-목표)
- [2. 아이디어 구성](#2-아이디어-구성)
  - [2-1. 전체 동작 Flow](#2-1-전체-동작-Flow)
  - [2-2. 각 보드 별 동작 Flow](#2-2-각-보드-별-동작-Flow)
- [3. 회로](#3-회로)
  - [3-1. CAD 회로](#3-1-CAD-회로)
  - [3-2. 실제 회로](#3-2-실제-회로)
- [4. 보드별 코드 및 통신 분석](#4-보드별-코드-및-통신-분석)
  - [4-1. Set(Atmega328p)](#4-1-SetAtmega328p)
  - [4-2. Display(STM32)](#4-2-DisplaySTM32)
- [5. 시연영상](#5-시연영상)

## 1. 프로젝트 목표
&nbsp;본 프로젝트의 목표는 아두이노와 STM32 보드를 여러 개 사용하여 링크를 구성하고 센싱 및 결과 출력을 하는 시스템을 설계하는 것을 목표로 합니다. <br/>
아두이노 Atmega328p의 레지스터 코딩, STM32의 부분 레지스터 코딩을 이용합니다. <br/>
또한, 링크구성에 통신수단인 UART, USART, I2C, SPI, CAN 등의 여러가지 통신을 이용하여 조별 37종 센서를 지급받아 많은 센서를 사용하는 것을 목표로 합니다. <br/>

&nbsp;이번 설계에서 지급받은 37종 센서 중 다음와 기존의 키트 센서 중 다음 센서들과 모듈들을 사용하여, 아이디어를 구성하였습니다.

1) Rotary Encoder: 엔코더를 누르거나 돌릴 때마다 LCD 디스플레이가 변경됩니다. 이 기능은 세탁 세부 설정을 합니다.
2) I2C 모듈을 달은 LCD: 엔코더를 누르거나 돌릴 때마다 LCD 디스플레이가 변경됩니다.
3) DC Motor: 세탁 메뉴 세부 설정에 따라 모터가 그 기능에 맞게끔 돌아갑니다.
4) RGB LED: 세탁 메뉴에 따라 색이 다르게 나오게끔 설정했습니다.
5) Passive Buzzer: 타이머 동작이 끝이나면 노래가 재생됩니다.

## 2. 아이디어 구현
<p align="center" style="margin: 20px 0;">
  <img width="49%" alt="Idea Image 1" src="https://github.com/user-attachments/assets/5edbf8bc-7e04-4ace-9c17-0264b54cd2d3" />
  <img width="49%" alt="Idea Image 2" src="https://github.com/user-attachments/assets/1ebfd5ae-860c-40dc-a3f0-7ecb313115a0" />
</p>

&nbsp;본 프로젝트의 설계 스케치와 동작과정을 간략히 구상한 것입니다. <br/>
이는 센서 중, 로타리 엔코더에 영감을 받아 작성한 스케치로 전체적인 구성은 **세탁기**입니다. <br/>
링크 구성은 로터리 엔코더를 조작하여 세탁기의 초기 설정을 하고 세탁기 동작 후(DC Motor) 세탁종료 후 알림까지의 링크 구성입니다. <br/>

### 2.1 전체 동작 Flow
- 로타리 엔코더를 꾹 눌러 전원 ON.
- LCD에서 바로 세탁모드를 띄워줌. (세탁모드 Default: Regular Wash)
- 엔코더를 돌려서 세탁모드를 변경. 이때 LCD에서 변경되는 세탁모드를 출력.(Regular Wash(표준세탁), Quick Wash(쾌속세탁))
  - Regular Wash, Quick Wash 두가지 설정. Quick Wash에서 엔코더 돌리면, Regular Wash로 다시 돌아감.
- 엔코더 버튼을 누를 시, 해당 세탁모드로 선택이 되어 다음 물 온도 설정 모드를 LCD에서 띄워줌. (30도, 40도, 60도)
- 위와 마찬가지 Mechanism으로 선택가능.
- 세탁모드(Regular Wash, Quick Wash) - 물 온도 설정 모드(30, 40, 60) - 헹굼 횟수(3, 4, 5) - 탈수 시간(Regular Wash : 30, 35, 40 / Quick Wash : 15, 20, 25) 선택
- 이 후, 세탁기 드럼통(DC Motor)가 해당 데이터를 받아 돌아가기 시작함. 이 때, RGB LED에 해당 드럼통(DC Motor)의 세탁 상태를 표시.
  - RGB LED : 세탁 - 노란색, 헹굼 - 초록색, 탈수 - 파란색, 종료 - 빨간색
- 드럼통(DC Motor)은 PWM이 빨라졌다 느려지는게 1주기.
  - 세탁 : 1주기, 헹굼 : 3 ~ 5주기, 탈수 : 세탁모드에 따라 다른 시간
- LCD에서는 남은 시간 표시.
- 탈수가 끝난 후, 부저에서 종료음 출력.

### 2.2 각 보드 별 동작 Flow
1) `Set(Atmega328p)`
  - 로타리 엔코더의 조작 부분.
  - 세탁설정(표준, 쾌속) - 물 온도 설정(30, 40, 60) - 헹굼 횟수(3, 4, 5) - 탈수 시간(표준 : 30, 35, 40/쾌속 : 15, 20, 25)를 설정하여 데이터를 전송하는 부분.
  - 엔코더의 기능은 버튼 + 돌리기이므로, 세가지를 활용하여 데이터 선택.
  - 데이터 `Display`와 `Drum`으로 전송.
  - `Display`로의 데이터는 로터리를 돌리면 모드 선택이 LCD에 띄워져야 하기 때문에 지속적으로 전송. 

2) `Display(STM32)`
  - `Set`에서 전달된 데이터를 LCD에 출력하는 부분.
  - `Set`의 로타리 엔코더의 버튼을 누를시 다음 선택으로 넘어가도록 설계.
  - `Set`의 로타리 엔코더를 돌리면 선택지가 LCD에 계속적으로 바뀌도록 설계.
  - `Set`에서의 네가지의 선택 모드가 결정이 되면, 해당하는 모드에서의 남은 시간을 띄워줌.

3) `Drum(Atmega328p)`
  - 해당하는 모드의 데이터를 전송받아, 세탁, 헹굼, 탈수의 주기와 시간을 결정.
  - Register FastPWM을 사용하여 조정.
  - 탈수에서의 시간은 세탁 Regular Wash, Quick Wash마다 시간을 다르게 설정.
  - 각 단계마다의 정보를 `Inform`에 전달하여, `Inform`의 LED RGB가 설정되도록 설계.
  - 탈수가 끝났을 시 `Inform`으로 정보를 전달하여, 종료 부저음이 출력되도록 설계.

4) `Inform(STM32)`
  - `Drum`에서 각 단계마다의 정보를 받아, LED RGB가 설정되도록 설계.
  - 탈수가 끝났을 시, 종료 부저음이 출력되도록 설계.

## 3. 회로
### 3.1 CAD 회로
<p align="center" style="margin: 20px 0;">
  <img width="90%" alt="CAD Image" src="https://github.com/user-attachments/assets/2dc3faa3-c67e-45a2-96a3-923f7bd07448" />
</p>

### 3.2 실제 회로
<p align="center" style="margin: 20px 0;">
  <img width="90%" alt="실제 회로 Image" src="https://github.com/user-attachments/assets/cd3008ad-d133-4b20-9c47-ba5fcbd05d31" />
</p>

## 4. 보드별 코드 및 통신 분석

> **해당 프로젝트에서는 `Set(Atmega328p)`, `Display(STM32)` 보드 개발 역할을 맡았습니다.** <br/>
> **해당 보드에 대한 설명만 있습니다.** <br/>
> **또한, Source 코드도 해당 보드에 대한 코드만 있습니다.** <br/>

### 4.1 Set(Atmega328p)
&nbsp;본 프로젝트를 구현하기 위해선, 로타리 엔코더와 LCD와의 데이터 전송이 이루어져야 하고, DC Motor로의 데이터 전송이 이루어져야 합니다. <br/>
로타리 엔코더와 LCD와의 데이터 전송은 로타리를 돌릴 때, LCD의 출력이 빨라야 하기 때문에 SPI 통신을 사용한다. 해당하는 통신을 위한 설정은 다음과 같습니다. <br/>

``` ino
...
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
...
```

&nbsp;LCD가 연결된 STM과 DC Motor가 연결된 Arduino로 SPI 통신을 합니다. <br/>
이 때, 본 Atmega328p 보드는 Master로서, 두개의 Slave와 데이터 통신을 하게 됩니다. <br/> 
이는 앞서 언급했듯이, 로터리 회전의 빠른 변화를 LCD가 채야하기 때문에 선택한 통신이라고 볼 수 있습니다. <br/>
SPI는 고속 데이터 전송에 적합한데, 클럭 속도와 데이터 프로토콜을 최적화 할 수 있어 높은 전송 속도를 지원합니다. <br/> 
따라서, SPI는 시간에 민감한 응용 프로그램에서 효율적으로 데이터를 송수신할 수 있다는 장점을 이용한 것입니다. <br/>
다만, 많은 핀을 사용한다는 단점이 있지만, 센서의 사용이 한개 뿐이기 때문에, 가능한 조건입니다. <br/> 
위에서, 두개의 Slave에 데이터를 전달 해야 하기 때문에, 다른 Atmega328p보드의 SS핀을 9번 핀(PB9)으로 설정하였습니다. <br/> 
또한, 위에서 로타리 버튼입력으로 인한 STM으로의 Data 전송(Polling 방식)과, 로타리 회전입력으로 인한 STM으로의 Data 전송(ISR 방식)이 동시에 발동 될 수 있어 LCD에 연결한 STM으로 보내는 Data는 STM과 연결된 MOSI핀에서 Data Loss가 일어날 수도 있습니다. <br/>
따라서, STM에서 제대로 데이터를 받았다는 ACK 신호를 보내어 로타리 엔코더의 다음 동작이 일어날 수 있게
설계하였습니다. <br/>

``` ino
...
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
...
```

&nbsp;본 코드에서 로터리 회전에 따른 데이터전송을 외부 인터럽트를 활용하여 전송하도록 하였습니다. <br/>
다만, 로터리 회전의 신호 인식이 빠르고 불안정한 관계로 `encoderValue`가 Overflow가 나는 경우가 발생합니다. <br/>
따라서, 이를 방지하기 위해 +2, +3을 하여, 오차를 줄였습니다. <br/>
기본적인 데이터 전송 Sheet는 다음과 같습니다. <br/>

| **encoderValue** | **세탁** | **물온도** | **헹굼** | **탈수** |
|:---:|:---:|:---:|:---:|:---:|
| 0 | Regular Wash | 30 | 3 | R: 30, Q: 15 |
| 1 | Quick Wash | 40 | 4 | R: 35, Q: 20 |
| 2 |  | 60 | 5 | R: 40, Q: 25 |

``` ino
...
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
...
```

&nbsp;초기의 버튼을 3초간 눌렀을 때의, LCD 화면 켜짐을 작성한 코드 입니다.

``` ino
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
...
```

&nbsp;`sendingStateData[5]`는 해당 데이터를 STM으로 전송, 이는 단계를 나타냅니다. <br/>

``` ino
...
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
...
```

&nbsp;Loop문 내부. <br/>
STM에서의 ACK신호를 받아 다음 단계로 동작하도록 하기 위한 Data 씹힘 현상을 방지합니다. <br/>

``` ino
...
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
...
```

&nbsp;마지막 탈수 모드를 선택하고 세탁이 가동하는 동안에는 엔코더의 동작을 멈추게 설계하였습니다. <br/>
이는 STM에서의 끝났다는 신호(ACK)을 사용하여 다시 세탁모드를 선택하는 단계로 돌아가도록 설계하였습니다. <br/>

### 4.2 Display(STM32)
&nbsp;다음은 `STM32CubeMX`에서의 설정입니다. <br/>

<p align="center" style="margin: 20px 0;">
  <img width="90%" alt="Pin Map Image" src="https://github.com/user-attachments/assets/62b70d75-4f23-401c-ab16-9edfc8aba30f" />
</p>

&nbsp;로터리 엔코더와의 SPI통신이 필요하기 때문에, `PA4` ~ `PA7`까지의 4개의 SPI1핀이 활성화가 되어있습니다. <br/>
또한, I2C 모듈을 이용한 LCD의 작업이 필요하기 때문에 `PB6`, `PB7`의 `I2C1`핀이 활성화가 되어있습니다. <br/>

<p align="center" style="margin: 20px 0;">
  <img width="90%" alt="Document Image 1" src="https://github.com/user-attachments/assets/479861fc-0fe9-4921-b9ce-73b55ea36d98" />
</p>
<img width="90%" alt="Document Image 2" src="https://github.com/user-attachments/assets/4c73bf03-0048-4198-a8fb-3f16980c9bbd" />
<img width="90%" alt="Document Image 3" src="https://github.com/user-attachments/assets/17c5fc4e-af9c-4339-bd49-bf6b53158cbe" />

&nbsp;다음은 TIM2의 Counter와 Prescaler의 Value Data Sheet입니다. <br/>
보면 65535까지 카운터가 가능한 것을 알 수 있습니다. <br/>
탈수 선택 후, 타이머를 띄우기 위한 설정으로, Timer Overflow Interrupt를 활용한 설계를 하였습니다. <br/>

<p align="center" style="margin: 20px 0;">
  <img width="49%" alt="Setting Image 1" src="https://github.com/user-attachments/assets/895a1a46-dcca-45ab-85e9-ab4d97ce2617" />
  <img width="49%" alt="Setting Image 2" src="https://github.com/user-attachments/assets/5109e609-52ae-42b6-b4cc-a1ca4b2dacd3" />
</p>

&nbsp;어떻게 보면, 좋지 않은 방법이긴 하지만, 최대 Clock 72Mhz까지 가능한 nucleoF103RB보드에서 Clock 64Mhz로 떨어뜨리는 행동은 좋지 않다고 생각합니다. <br/>
다만, Prescaler를 7999로 설정하여 64Mhz/7999+1 = 8000hz로 Counter Clock을 설정한 뒤, 8000hz로 Counter의 주기를 9999까지 가져가 1.25초 대략 1초마다 Timer Overflow Interrupt가 발생하도록 설정하였습니다. <br/>

``` c
...
#include "i2c_lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
   uint8_t data_t[4];
   data_u = (cmd&0xf0);
   data_l = ((cmd<<4)&0xf0);
   data_t[0] = data_u|0x0C;  //en=1, rs=0
   data_t[1] = data_u|0x08;  //en=0, rs=0
   data_t[2] = data_l|0x0C;  //en=1, rs=0
   data_t[3] = data_l|0x08;  //en=0, rs=0
   HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
...
```

``` c
/*
 * i2c_lcd.h
 *
 *  Created on: Oct 27, 2024
 *      Author: root
 */

#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f1xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

#endif /* INC_I2C_LCD_H_ */
```

&nbsp;i2c_lcd 라이브러리를 이용하여 LCD의 출력을 설계하였습니다. <br/>

``` c

```
