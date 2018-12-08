/** This is just a simple program to turn on/off rpi gpio pins
 * using the Wiring Pi library for debug purposes. It lets us make
 * sure hardware connections are sound.
 */

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>

int main(const int argc, const char* argv[]) {
  if(argc != 3) {
    printf("usage: ./%s [pin #] [HIGH or LOW]\n", argv[0]);
    return 0;
  }
  const int pin = atoi(argv[1]);
  const int val = strcmp(argv[2], "HIGH")==0 || atoi(argv[2])==1;

  if(wiringPiSetupGpio()<0) {
    printf("failed to init wiring pi lib\n");
    return -1;
  }

  pinMode(pin, OUTPUT);
  digitalWrite(pin, val);
  return 0;
}
