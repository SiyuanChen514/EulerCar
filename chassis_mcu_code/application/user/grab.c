#include "grab.h"


void lift(void){
    _close(100, 650);
    up(50, 650);
}


void lay(void){
    down(50, 350);
    open(30, 500);
}