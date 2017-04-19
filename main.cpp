#include "mbed.h"
//#include "NanostackInterface.h"

DigitalOut led(LED1);

int main() {
    
    while(true)
    {
        led = 1;
        wait(2.0);
        led = 0;
        wait(1);
    }
}
