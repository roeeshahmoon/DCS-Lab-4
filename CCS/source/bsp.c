#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  // LCD configuration
  LCD_DATA_WRITE &= ~0xFF;
  LCD_DATA_DIR |= 0xF0;    // P1.4-P1.7 To Output('1')
  LCD_DATA_SEL &= ~0xF0;   // Bit clear P1.4-P1.7
  LCD_CTL_SEL  &= ~0xE0;   // Bit clear P2.5-P2.7
  // LED configuration
  //LEDsArrPortSel &= ~0xFF;
 // LEDsArrPort  &= ~0xFF;
 // LEDsArrPortDir |= 0xFF;
  

  // Generator Setup
  //From the table at CCIx p2.4
  GenPortDir &=  ~BIT3;               // P2.3 Input Capture = '0'
  GenPortSel |=  BIT3;              // P2.3 Select = '1'




// RGB P1.0-P1.2 setup

   RGBPortSel &= ~0x07; // set P1.0 - P1.2 to I/O
   RGBPortDir |= 0x07;// set P1.0 - P1.2 to Output
   RGBPortOut &= ~0x07; // Reset P1.0 - P1.2 Output
// Buzzer P2.4 setup
   BuzzPortSel |= 0x04; // set P2.2 to I/O
   BuzzPortDir |= 0x04;// set P2.2 to Output
   BuzzPortOut &= ~0x04; // Reset P2.2 Output
// UART INIT
   //P2DIR = 0xFF;                             // All P2.x outputs
  // P2OUT = 0;                                // All P2.x reset
  // P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  // P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  // P1DIR |= RXLED + TXLED;
 //  P1OUT &= 0x00;
//LDR
   LDRPortSel &= ~BIT0;
   LDRPortDir &= ~BIT0;
  _BIS_SR(GIE);                     // enable interrupts globally


}




