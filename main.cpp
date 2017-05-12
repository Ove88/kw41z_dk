#include "mbed-os/mbed.h"
#include "NanostackRfPhyKw41z.h"
#include "ThreadInterface.h"

DigitalOut led = LED1;
InterruptIn btn = SW3;

NanostackRfPhyKw41z rfPhy;
ThreadInterface mesh;

void trace_printer(const char* str) 
{
    printf("%s\n", str);
}

void threadConnect();

int main() 
{
    mbed_trace_init();
    mbed_trace_print_function_set(trace_printer);

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
   printf("\n\nConnecting...\n");
    mesh.initialize(&rfPhy);

    if (mesh.connect()) {
        printf("Connection failed!\n");
        return;
    }

    while (NULL == mesh.get_ip_address())
        Thread::wait(500);

    printf("connected. IP = %s\n", mesh.get_ip_address());
}