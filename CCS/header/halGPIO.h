#ifndef _halGPIO_H_
#define _halGPIO_H_


#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer

extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern void  get_KB(char* p);
extern void startTimerA(int X);
extern void timer_delay();
extern void RGB_clear();
extern void  LPM0_GIE_SET();
extern void stopTImerA();
extern void UpdatePWM(int x);
extern void sysConfig(void);
extern void SetByteToPort(char); // Added By RK
extern void clrPortByte(char);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void write_freq_tmp_LCD();
extern void write_signal_shape_tmp_LCD();
extern void ADC_config(void);
extern void StartSamplingADC12( unsigned int* p_adc);
extern void mult(signed short operand1, signed short operand2, unsigned long* result);
extern void div(unsigned long Divided, unsigned short Divisor, unsigned long* Quotient, unsigned long* Remainder);
extern void StartTimerB();
extern void get_X();
extern __interrupt void PBs_handler(void);
extern __interrupt void PBs_handler_P2(void);
extern __interrupt void Timer_A(void);
extern __interrupt void Timer_A1(void);
extern __interrupt void ADC12_ISR(void);
extern __interrupt void USART1_RX (void);
extern __interrupt void USART1_TX (void);


#endif

// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~0X40) : (P2OUT|=0X40)) // P2.6 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X40) : (P2DIR|=0X40)) // P2.6 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~0X80) : (P2OUT|=0X80)) // P2.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X80) : (P2DIR|=0X80)) // P2.7 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off          lcd_cmd(0x0C)
#define cursor_on           lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line        lcd_cmd(0xC0)
#define DMA2_OFF()         DMA2CTL &= ~DMAEN
#define GIE_RESET_PUSHBUTTON()    P1IE &= ~0x0f;
#define GIE_RESET_PUSHBUTTON4()    P2IE &= ~0x01;
extern void StopTimerB();
extern void StopAllTimers();
extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void intToStr(int num,char* strnum);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
extern void RGB_color(const int x);
/*
 *  Delay functions for HI-TECH C on the PIC18
 *
 *  Functions available:
 *      DelayUs(x)  Delay specified number of microseconds
 *      DelayMs(x)  Delay specified number of milliseconds
*/






