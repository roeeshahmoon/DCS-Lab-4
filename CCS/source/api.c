#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"


// Global Variables


//-------------------------------------------------------------
//                         RGB Blink
//-------------------------------------------------------------

void Blink_RGB(){
    const int RGB_Arr[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
    unsigned int i = 0 ;
    StopAllTimers();
    lcd_clear();

    while(state == state1){
        disable_interrupts();
        RGB_color(RGB_Arr[i]);
        timer_delay();
        if(++i == 8){
            i = 0;
        }
    }
    RGB_clear();


}

//-------------------------------------------------------------
//                         Count Up
//-------------------------------------------------------------
void CountUp( int* P_Counter){
    char Str_Counter[16];
    char const * StrShowCounter ="The counter is:";
    StopAllTimers();
    lcd_clear();
    lcd_puts(StrShowCounter);
    startTimerA(1024);
    while(state == state2){
    disable_interrupts();
    lcd_new_line;
    intToStr(*P_Counter,Str_Counter);
    lcd_puts(Str_Counter);
    enable_interrupts();
    timer_delay();
    (*P_Counter)+=1;
    }
}

//-------------------------------------------------------------
//                         Sounds into Buzzer
//-------------------------------------------------------------
void Cricular_tone(){
    const int tone_series[7] = {1048,839,699,599,524,466,419}; // {1kHz, 1.25kHz, 1.5kHz, 1.75kHz, 2kHz, 2.25kHz, 2.5kHz}
    unsigned int i = 0 ;
    StopAllTimers();
    lcd_clear();
    StartTimerB(); //using this module to create pwm to the buzzer
    while(state == state3){

        UpdatePWM(tone_series[i]);
        timer_delay();
      if(++i == 7){
            i = 0;
        }
    }
    StopTimerB();
}


//-------------------------------------------------------------
//                         Set X
//-------------------------------------------------------------
void Set_X(){
    StopAllTimers();
    lcd_clear();
    get_X();
    state = state8;
}

//-------------------------------------------------------------
//                         LDR_Volt
//-------------------------------------------------------------


void LDR_Volt(){
    char strnum[20];
    unsigned long* P_Quotient ,*P_Resdiue,Quotient,Resdiue,*P_Result,Result ;
    unsigned int  ADC_S,* P_ADC=&ADC_S,ADC_Delta=1241;
    while(state == state5){
        StartSamplingADC12(P_ADC);
        P_Resdiue = &Resdiue;
        P_Quotient = &Quotient;
        P_Result = &Result;
        mult(ADC_S, 100 ,P_Result);
        div(Result,ADC_Delta,P_Quotient,P_Resdiue);
        intToStr(Quotient,strnum);
        if(Quotient<10){
            strnum[2] = strnum[0];
            strnum[0] = '0';
            strnum[1] = '0';
        }
        else if(Quotient<100){
            strnum[2] = strnum[1];
            strnum[1] = strnum[0];
            strnum[0] = '0';
        }



        lcd_clear();
        lcd_home();
        lcd_puts("LDR Value is:");
        lcd_new_line;
        lcd_data(strnum[0]);
        lcd_data('.');
        lcd_data(strnum[1]);
        lcd_data(strnum[2]);
        lcd_data('[');
        lcd_data('v');
        lcd_data(']');
        startTimerA(500);
        stopTImerA();
    }
}
