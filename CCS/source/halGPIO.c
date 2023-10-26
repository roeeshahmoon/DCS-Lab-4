#include  "../header/halGPIO.h"     // private library - HAL layer

// Global Variables
char string[5];
int j = 0;
int update_X_flag = 0;
int X=500;


//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){ 
	GPIOconfig();
	USCI_CONFING();
	lcd_init();
	ADC_config();

}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
// 				Set Byte to Port
//--------------------------------------------------------------------
void SetByteToPort(char ch){
	PBsArrPortOut |= ch;  
} 
//--------------------------------------------------------------------
// 				Clear Port Byte
//--------------------------------------------------------------------
void clrPortByte(char ch){
	PBsArrPortOut &= ~ch;
} 
//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){  //
	volatile unsigned int i;
	
	for(i=t; i>0; i--);
}

//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}

//---------------------------------------------------------------------
//            LCD
//---------------------------------------------------------------------
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
    while(*s)
        lcd_data(*s++);
}
//******************************************************************
//    write frequency template to LCD
//******************************************************************
void write_freq_tmp_LCD(){
   lcd_clear();
   lcd_home();
    const char SquareWaveFreq[] = "fin=";
    const char Hz[] = "Hz";
     lcd_puts(SquareWaveFreq);
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_puts(Hz);
}
//******************************************************************
//    write signal shape template to LCD
//******************************************************************
void write_signal_shape_tmp_LCD(){
   lcd_clear();
   lcd_home();
    const char signal_shape[] = "signal shape: ";
     lcd_puts(signal_shape);
     lcd_new_line;
}
//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}

//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec

}

//******************************************************************
// mult and div functions q-format
//******************************************************************

void div(unsigned long Divided, unsigned short  Divisor, unsigned long* Quotient, unsigned long* Remainder) {
    unsigned long R9 = 32;
    unsigned long R4 = Divided;
    unsigned long R5 = 0;
    unsigned short R6 = Divisor;
	unsigned short median = Divisor >> 1;
    unsigned long R8 = 0;
    unsigned long carry = 0;
    while (R9 > 0) {
        carry = (R4 & 0x80000000) >> 31;
        R4 = (R4 << 1) ;
        R5 = (R5 << 1) ;
        R5 |= carry;
        if (R6 > R5) {
            R8 = (R8 << 1) ;
        }
        else {
            R5 -= R6;
            carry = 1;
            R8 = (R8 << 1) ;
            R8 |= carry;

        }

        R9--;

    }
    if (R5 > median) {
            R8++;
        }
    *Quotient = R8;
    *Remainder = R5;

}


void mult(signed short operand1, signed short operand2, unsigned long* result) {
    unsigned short R8 = 1;
    unsigned long temp1=0;
    unsigned short R4 = operand1;
    unsigned short R5 = operand2;
    unsigned short R6 = 0;
    unsigned long R7 = 0;
    unsigned short carry = 0;
    while (R8 != 0) {
        if (R8 & R5) {
            R7 += R4;
        }
        carry = R7 & 0x1;
        R6 = R6 >> 1;
        R6 = R6 | (carry << 15);
        R7 = R7 >> 1;

        R8 = R8 << 1;

    }
    R6 = R6;
    R7 = R7 << 16;
    temp1 = (unsigned long)R6;
    *result = R7 + temp1;



}

void LPM0_GIE_SET(){
    __bis_SR_register(LPM0_bits + GIE);
}


void get_X(){
    int X_3 = 0,X_2=0,X_1=0,X_0=0,k=0,temp,len = 0;
    int* P_3 = &X_3;
    int* P_2 = &X_2;
    int* P_1 = &X_1;
    while(string[len] != '\n'){
        len++;
    }
    while(string[k] != '\n'){
        temp = (int)(string[len-k-1]) - '0';
        if(k==0){
            X_0 = temp;
            X = X_0;
        }
        else if(k==1){
            mult(temp,10,P_1);
            X += X_1;
        }
        else if(k==2){
            mult(temp,100,P_2);
            X += X_2;
        }
        else if(k==3){
            mult(temp,1000,P_3);
            X += X_3;
        }
        k++;
    }


}



//-------------------------------------------------------------
//                         intToStr
//-------------------------------------------------------------

void intToStr(int num,char* strnum) {
    int i = 0, lenNum = 1, tempNum = num;
    while (tempNum > 9) {
        tempNum /= 10;
        lenNum++;
    }
    strnum[lenNum - 1 - i++] = '0' + (num % 10);
    num /= 10;
    while (num > 0) {
        strnum[lenNum - 1 - i++] = '0' + (num % 10);
        num /= 10;
    }
    strnum[lenNum] = '\0' ;
}





//******************************************************************
// Start timer A functions
//******************************************************************
void startTimerA(int ms){
    TACCR0 = ms<<2;  // Timer Cycles - max/2
    TA0CTL = TASSEL_1 + MC_1 + ID_3+TAIE;  //  select: 1 -ACLK ; control: 1 - Up  ; divider: 3 - /8
    TACCTL0 = CCIE;
    TACCTL1 = 0;
    TACCTL2 = 0;


    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
}
//---------------------- Timer delay---------------------------------
void timer_delay(){

    startTimerA(X);

}
void stopTImerA(){
    TA0CTL = MC_0+TACLR;

}



void StopAllTimers(){
    stopTImerA(); // halt delay Timer

}

//******************************************************************
// timer B functions
//******************************************************************



void StartTimerB(){
    TBCTL = TBSSEL_2 + MC_1 + TBCLR;;                  //  SMCLK, upmode,
    TBCCTL1 =  OUTMOD_7; // TBCCR1 reset/set;
}
void StopTimerB(){
    TBCTL =  MC_0 + TBCLR;                  //  SMCLK, upmode,
}

void UpdatePWM(int x){
    TBCCR0 = x;
    TBCCR1 = x>>1;// x/2 to set 50 % dutycycle
}
//******************************************************************
// ADC functions
//******************************************************************
void StartSamplingADC12( unsigned int* p_adc){
    ADC12CTL0 |= ENC + ADC12SC;               // Start sampling
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    ADC12CTL0 &= ~ADC12ON; // Don't get into interrupt
    *p_adc = ADC12MEM0;
}
//******************************************************************
// RGB functions
//******************************************************************


void RGB_color(const int x){
    RGBPortOut = x;
}

void RGB_clear(){
    RGBPortOut = 0;
}







//*********************************************************************
//            Port1 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT1_VECTOR
  __interrupt void PBs_handler(void){
   
	delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------
	if(PBsArrIntPend & PB0){
	  PBsArrIntPend &= ~PB0;
        }
        else if(PBsArrIntPend & PB1){
	  PBsArrIntPend &= ~PB1; 
        }
	else if(PBsArrIntPend & PB2){ 
	  PBsArrIntPend &= ~PB2;
        }
//---------------------------------------------------------------------
//            Exit from a given LPM 
//---------------------------------------------------------------------	
        switch(lpm_mode){
		case mode0:
		 LPM0_EXIT; // must be called from ISR only
		 break;
		 
		case mode1:
		 LPM1_EXIT; // must be called from ISR only
		 break;
		 
		case mode2:
		 LPM2_EXIT; // must be called from ISR only
		 break;
                 
                case mode3:
		 LPM3_EXIT; // must be called from ISR only
		 break;
                 
                case mode4:
		 LPM4_EXIT; // must be called from ISR only
		 break;
	}
        
}







  //-------------------------------------------------------------------------------------
  //            ADC configuration
  //-------------------------------------------------------------------------------------
  void ADC_config(void){
      ADC12CTL0 = SHT0_2 + ADC12ON;             // Sampling time, ADC12 on
      ADC12CTL1 = SHP;                          // Use sampling timer
      ADC12IE = 0x01;                         // P1.3 ADC option select
  }



//-------------------------------------------------------------------------------------
//           USCI configuration
//-------------------------------------------------------------------------------------


void USCI_CONFING(){

  volatile unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  FLL_CTL0 |= XCAP14PF;                     // Configure load caps

    do
    {
    IFG1 &= ~OFIFG;                           // Clear OSCFault flag
    for (i = 0x47FF; i > 0; i--);             // Time for flag to set
    }
    while ((IFG1 & OFIFG));                   // OSCFault flag still set?

    P4SEL |= 0x03;                            // P4.1,0 = USART1 TXD/RXD
    ME2 |= UTXE1 + URXE1;                     // Enable USART1 TXD/RXD
    U1CTL |= CHAR;                            // 8-bit character
    U1TCTL |= SSEL0;                          // UCLK = ACLK
    U1BR0 = 0x03;                             // 32k/9600 - 3.41
    U1BR1 = 0x00;                             //
    U1MCTL = 0x4A;                            // Modulation
    U1CTL &= ~SWRST;                          // Initialize USART state machine
    IE2 |= URXIE1;                            // Enable USART1

}



//*********************************************************************
//-------------------------------------------------------------------------------------
//           USCI ISR
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
//                           TX ISR
//-------------------------------------------------------------------------------------

#pragma vector=USART1TX_VECTOR
__interrupt void USART1_TX (void)

{
    if(state == state7) TXBUF1 = '7'; //print menu in PC
 //   else if (state == state7) UCA0TXBUF = '7';
    else if (update_X_flag) TXBUF1 = '4';
    else TXBUF1 = 'F';   // Finish
    IE2 &= ~UTXIE1;                       // Disable USCI_A0 TX interrupt
}
// USCI A0/B0 Receive ISR
#pragma vector=USART1RX_VECTOR
__interrupt void USART1_RX (void)
{
    if(RXBUF1 == '1' && update_X_flag == 0){
        state = state1;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '2' && update_X_flag == 0){
        state = state2;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '3' && update_X_flag == 0){
        state = state3;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '4' || update_X_flag){

        if (update_X_flag == 1){
            string[j] = RXBUF1;
            j++;
            if (string[j-1] == '\n'){
                j = 0;
                update_X_flag = 0;
                state = state4;
                IE2 |= UTXIE1;       // Enable USCI_A0 TX interrupt

            }
        }
        else{
        update_X_flag = 1;
        IE2 |= UTXIE1;        // Enable USCI_A0 TX interrupt
        }

    }
    else if(RXBUF1 == '5' && update_X_flag == 0){
        state = state5;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '6' && update_X_flag == 0){
        state = state6;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '7' && update_X_flag == 0){ //RealTime
        state = state7;
        IE2 |= UTXIE1;
    }
    else if(RXBUF1 == '8' && update_X_flag== 0){
        state = state8;
        IE2 |= UTXIE1;                        // Enable USCI_A0 TX interrupt
    }
    //else if(UCA0RXBUF == '9' && update_X_flag == 0){//Real time
        //state = state9;
        //IE2 |= UCA0TXIE;
  //  }

 //---------------------------------------------------------------------
 //            Exit from a given LPM
 //---------------------------------------------------------------------
             switch(lpm_mode){
             case mode0:
                 LPM0_EXIT; // must be called from ISR only
                 break;

             case mode1:
                 LPM1_EXIT; // must be called from ISR only
                 break;

             case mode2:
                 LPM2_EXIT; // must be called from ISR only
                 break;

             case mode3:
                 LPM3_EXIT; // must be called from ISR only
                 break;

             case mode4:
                 LPM4_EXIT; // must be called from ISR only
                 break;
             }
}

//*********************************************************************
//            Port2 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler_P2(void){
      delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------

          PB3sArrIntPend = 0x00;

//---------------------------------------------------------------------
//            Exit from a given LPM
//---------------------------------------------------------------------
      switch(lpm_mode){
      case mode0:
          LPM0_EXIT; // must be called from ISR only
          break;

      case mode1:
          LPM1_EXIT; // must be called from ISR only
          break;

      case mode2:
          LPM2_EXIT; // must be called from ISR only
          break;

      case mode3:
          LPM3_EXIT; // must be called from ISR only
          break;

      case mode4:
          LPM4_EXIT; // must be called from ISR only
          break;
      }
  }
  //*********************************************************************
  //            TimerB Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_B1_VECTOR
  __interrupt void TIMER0_B1_ISR(void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_VECTOR (void)
  #else
  #error Compiler not supported!
  #endif
  {
    switch(__even_in_range(TBIV, 0x0A))
    {
        case  TBIV_NONE: break;              // Vector  0:  No interrupt
        case  TBIV_TBCCR1:                   // Vector  2:  TBCCR1 CCIFG
            TBCTL &= ~(TBIFG);
          break;
        case TBIV_TBCCR2: break;
        case TBIV_TBCCR3 : break;                  // Vector  6:  Reserved CCIFG
        case TBIV_TBCCR4: break;                  // Vector  8:  Reserved CCIFG
        case TBIV_TBIFG : break;              // Vector 10:  TAIFG
        default:  break;
    }
  }
  //*********************************************************************
  //            TimerA0 Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_A0_VECTOR
  __interrupt void Timer_A (void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
  #else
  #error Compiler not supported!
  #endif
  {

      LPM0_EXIT;
      TACTL = MC_0+TACLR;
  }
  //*********************************************************************
  //            TimerA1-2 Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_A1_VECTOR
  __interrupt void Timer_A1 (void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
  #else
  #error Compiler not supported!
  #endif
  {

      TACTL &= ~(TAIFG);
  }
  //*********************************************************************
  //            ADC12 Vector Interrupt Service Routine
  //*********************************************************************
  #pragma vector = ADC12_VECTOR
  __interrupt void ADC12_ISR(void)
  {
      if (ADC12MEM0>=0)       //this is pointless comparison only for debugger
      __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
  }

