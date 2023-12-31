#ifndef _bsp_H_
#define _bsp_H_

//#include  <msp430g2553.h>          // MSP430x2xx
#include  <msp430xG46x.h>  // MSP430x4xx


#define   debounceVal      250
//#define   LEDs_SHOW_RATE   0xFFFF  // 62_5ms

// RGB abstraction
#define RGBPortOut        P1OUT
#define RGBPortSel        P1SEL
#define RGBPortDir        P1DIR

// LEDs abstraction
//#define LEDsArrPort        P9OUT
//#define LEDsArrPortDir     P9DIR
//#define LEDsArrPortSel     P9SEL

// LCDs abstraction
#define LCD_DATA_WRITE     P1OUT
#define LCD_DATA_DIR       P1DIR
#define LCD_DATA_READ      P1IN
#define LCD_DATA_SEL       P1SEL
#define LCD_CTL_SEL        P2SEL

//   Generator abstraction
#define GenPort            P2IN
#define GenPortSel         P2SEL
#define GenPortDir         P2DIR
#define GenPortOut         P2OUT

//   Buzzer abstraction
#define BuzzPortSel        P2SEL
#define BuzzPortDir        P2DIR
#define BuzzPortOut        P2OUT

// PushButton 3 abstraction for Main Lab
#define PB3sArrPort         P2IN
#define PB3sArrIntPend      P2IFG
#define PB3sArrIntEn        P2IE
#define PB3sArrIntEdgeSel   P2IES
#define PB3sArrPortSel      P2SEL
#define PB3sArrPortDir      P2DIR
#define PB3sArrPortOut      P2OUT

// PushButtons abstraction
#define PBsArrPort	       P1IN
#define PBsArrIntPend	   P1IFG
#define PBsArrIntEn	       P1IE
#define PBsArrIntEdgeSel   P1IES
#define PBsArrPortSel      P1SEL
#define PBsArrPortDir      P1DIR
#define PBsArrPortOut      P1OUT
#define PB0                0x01   // P1.0
#define PB1                0x02  // P1.1
#define PB2                0x04  // P1.2
#define PB3                0x10  // P2.0
// LDR abstraction
#define LDRPortSel        P6SEL
#define LDRPortDir        P6DIR



// KeyPad abstraction
//#define KeyPadArrPort      P10IN
//#define KeyPadIntPend      P2IFG
//#define KeyPadIntEn        P2IE
//#define KeyPadIntEdgeSel   P2IES
//#define KeyPadPortSel      P10SEL
//#define KeyPadPortDir      P10DIR
//#define KeyPadPortOut      P10OUT

#define Col1               0x80
#define Col2               0x40
#define Col3               0x20
#define Col4               0x10
#define Row1               0x08
#define Row2               0x04
#define Row3               0x02
#define Row4               0x01
// UART abstraction
#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT0
#define RXD BIT1

//#define LedsClear()    P9OUT  &= ~0xFF  ;    //clear to Port 9

extern void GPIOconfig(void);
#endif



