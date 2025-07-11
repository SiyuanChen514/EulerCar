#include "display.h"
#include "math.h"
#include "ssd1306.h"
#include "string.h"


// volt: mv
void display_voltage(short volt){
    // short v = volt;
    // SSD1306_COLOR color = White;
    // int digit[5];
    // int idx = 0;
    // for(idx = 0; idx < 5; idx++){
    //     digit[idx] = v / (short)pow(10, 4 - idx);
    // }
    // char* vt = (char*) malloc(sizeof(char) * 6);
    // for(idx = 0; idx < 5; idx++){
    //     vt[idx] = digit[idx] + '0';
    // }
    // vt[5] = '\0';
    // ssd1306_DrawString(vt, Font_7x10, color);
    // ssd1306_UpdateScreen();
}