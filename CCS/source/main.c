#include  "../header/api.h"         // private library - API layer
#include  "../header/app.h"         // private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;

void main(void){
    int  Counter=0;
    int* P_Counter = &Counter;
    state = state8;  // start in idle state on RESET
    lpm_mode = mode0;     // start in idle state on RESET
    sysConfig();
    lcd_init();
    lcd_clear();


  while(1){
    switch(state){

      case state1: //char '1' from keyboard
          Blink_RGB();     //Blink RGB LED, color by color with delay of X[ms]
          break;

      case state2:        //char '2' from keyboard
          CountUp(P_Counter);      //Count up onto LCD screen with delay of X[ms]
          break;

      case state3: ;  //char '3' from keyboard
          Cricular_tone();     //Circular tone series via Buzzer with delay of X[ms]
          break;
                
      case state4: //char '4' from keyboard
          Set_X(); //Get delay time X[ms]:
          break;

      case state5: //char '5' from keyboard
          LDR_Volt();//LDR 3-digit value [v] onto LCD
          break;
      case state6:            //char '6' from keyboard
          lcd_clear();    //Clear LCD screen
          Counter = 0;
          enable_interrupts();
          state = state8;
          break;
      case state7:      //char '7' from keyboard
     //       Show_Menu();      //Show menu
          break;
      case state8: //char '8' from keyboard
          enable_interrupts();
          enterLPM(lpm_mode);
          break;
    }
  }
}

  
  
  
  
  
  
