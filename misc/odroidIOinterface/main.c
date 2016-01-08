//main.c
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include "hidapi.h"
#include "usb_io.h"

#define TRUE 1
#define FALSE 0

int main() {
   hid_device *dev;
   dev = dev_open();
   if(!dev)   return -1;
   
   printf("%s\n", module_version());
   printf("%s\n", rom_version(dev));//<- gives me crap
   
   printf("Toggeling LED\n");
   toggle_led(dev);
   
   printf("Push button!\n");
   while(!read_switch(dev)){} //<-stuck
   printf("Button pressed\n");
   
   return 0;
   }