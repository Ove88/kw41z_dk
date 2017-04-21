#include "mbed.h"

DigitalOut led = LED1;
InterruptIn btn = SW3;

NanostackRfPhyKw41z rfPhy;
ThreadInterface network;

void threadConnect();

int main() 
{
   threadConnect();

    while(true)
    {
       led = 1;
       wait(2.0);
       led = 0;
       wait(1);
    }
}

void threadConnect()
{
    rfPhy.rf_register();
    network.initialize(&rfPhy);
    network.connect();

}