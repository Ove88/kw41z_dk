#include "mbed.h"

DigitalOut led = {LED1};
InterruptIn g = {LED2};

ThreadInterface fg;
NanostackRfPhyKw41z rfPhy;
int main() {

    while(true)
    {
       led = 1;
       wait(2.0);
       led = 0;
       wait(1);
    }
}
