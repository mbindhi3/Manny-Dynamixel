 #include "mbed.h"
 #include "AX12.h"
 
 DigitalOut myled(LED1);
 
 int main() {
 
   AX12 myax12 (p9, p10, 11);
 
   while (1) {
       myax12.SetGoal(0);    // go to 0 degrees
       myled = 1;
       wait (5.0);
       myax12.SetGoal(300);  // go to 300 degrees
       myled = 0;
       wait (5.0);
   }
 }