#include "mbed.h"
#include "NanostackInterface.h"
#include "ThreadInterface.h"
#include "PhyInterface.h"



DigitalOut led(LED1);

int main() {
    
    while(true)
    {
        SystemCoreClock 
        led = 1;
        wait(2.0);
        led = 0;
        wait(1);
    }
}
