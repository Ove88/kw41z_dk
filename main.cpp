#include "mbed.h"
#include "NanostackInterface.h"
#include "ThreadInterface.h"
#include "PhyInterface.h"



DigitalOut led = {LED1};
InterruptIn g = {LED2};
Timer gf;
Thread th;

int main() {
    Phy_Init();
    while(true)
    {
       
       led = 1;
       wait(2.0);
       led = 0;
       wait(1);
    }
}
