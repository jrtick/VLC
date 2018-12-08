/** This is a helper program for debugging of our system.
 * It can read the current value of the ADC, print ADC speed stats,
 * or even write ADC values to a file for a certain amount of time.
 * This helped us look at timing for our project so that we could
 * hand decode bits to make sure they were being modulated correctly.
 */

#include <wiringPi.h>
#include "adc_lib.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef enum {READ, WRITE, STATS} ProgramMode;

int main(const int argc, const char* argv[]) {
  if(argc == 1 || argc>3) {
    printf("Usage: %s [read] OR [write] OR [stats]\n", argv[0]);
    return 0;
  }
  const ProgramMode mode = argc==1 || strcmp("read", argv[1])==0? READ : 
                           strcmp("write", argv[1])==0? WRITE : STATS;
  int sample_count = 1e5;
  float write_dur = 5000;
  if(mode == WRITE && argc==3) {
    const float suggestion = atof(argv[2]);
    if(suggestion>0 && suggestion < 10*60*1000) write_dur = suggestion;
    else printf("ignoring write dur '%s'\n", argv[2]);
  } else if (mode == STATS && argc==3) {
    const int suggestion = atoi(argv[2]);
    if(suggestion>0 && suggestion < 1e8) sample_count = suggestion;
    else printf("ignoring sample_count '%s'\n", argv[2]);
  }

  // setup hw control lib
  if(wiringPiSetup()<0) {
    printf("failed to initialize wiring pi\n");
    return -1;
  } else if(initADC()<0) {
    printf("failed to initialize wiring pi SPI lib\n");
    return -1;
  }

  volatile float unused;
  int i,j;
  unsigned start, stop;
  float duration, val;
  FILE* file;

  switch(mode) {
  case STATS: // measure sample speed
    for(i=0;i<8;i++) { // num iters
      start = micros();
      for(j=0;j<sample_count;j++) {
        unused = readADC();
      }
      stop = micros();
      printf("sample rate is %.3fHz\n",
             1e6*(((float)sample_count)/(stop-start)));
    }
    break;
  case WRITE:
    file = fopen("data.txt", "w");
    duration = 0;
    start = micros();
    while(duration < write_dur) {
      val = readADC();
      duration = (micros()-start)/1000.f;
      fprintf(file, "%f\t%f\n", duration, val); // milliseconds, volts
    }
    fclose(file);
    break;
  case READ:
    while(1) {
      val = 0;
      for(i=0;i<10;i++) val += readADC();
      printf("%.4f\n", val*0.1f);
      delay(100); // print at 10hz-ish
    }
    break;
  }

  return 0;
}
