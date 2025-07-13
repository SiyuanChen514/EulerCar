#include "grab.h"


void lift(void){
    _close(100, 650);
    up(50, 650);
}


void lay(void){
    open(30, 500);
    down(50, 400);
}