// includes
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <wiringPi.h>
#include "adc_lib.h"
#include <stdbool.h>
#include <pthread.h>
#include "ids.h"

// random defines
#define STR(x) #x
#define ASSERT(cond) \
  if(!(cond)) { \
    printf("assertion '%s' failed at line %d\n", STR(cond), __LINE__); \
    exit(-1); \
  }

// hw defines
#define LED_PIN 25
#define SAMPLE_PERIOD_US 25
#define PPM_BITS 1
#define PPM_SLOT_COUNT (1<<(PPM_BITS))
#define PPM_SLOT_US (SAMPLE_PERIOD_US*10)
#define PPM_PERIOD_US (PPM_SLOT_US*PPM_SLOT_COUNT)

// Protocol defines
#define PREAMBLE  0b01010101
#define POSTAMBLE 0b00100100
#define MAX_MSG_SIZE 64 // bytes

int send_OOK(const char* buf, const int len) {
  for(int i=0;i<len;i++) {
    digitalWrite(LED_PIN, buf[i]);
    delayMicroseconds(PPM_PERIOD_US);
  }
  // TODO: FAST SENSING
  digitalWrite(LED_PIN, 0);
  return 0;
}

/** this is Manchester encoding if PPM_BITS==1 */
int send_PPM(const char* buf, const int byte_count) {
  for(int i=0;i<byte_count;i++) {
    const char cur_char = buf[i];
    for(int j=0;j<8/PPM_BITS;j++) {
      const char cur_val = (cur_char>>(j*PPM_BITS)) & ((1<<PPM_BITS)-1);
      for(int k=0;k<PPM_SLOT_COUNT;k++) {
        digitalWrite(LED_PIN, (k==cur_val));
        delayMicroseconds(PPM_SLOT_US);
      }
    }
  }
  digitalWrite(LED_PIN, 0);
  return 0;
}
// receives one period of PPM
inline char receivePPM() {
  const unsigned start = micros();
  unsigned duration;
  while((duration=micros()-start) < PPM_PERIOD_US) {
    
  }
}

int receive_OOK(char* buf, const int max_bytes) {
  
}

int send(const char* msg, const int msg_len,
         const char to_addr, const char from_addr) {
  ASSERT(msg_len < MAX_MSG_SIZE);
  ASSERT(to_addr < 16);
  ASSERT(from_addr < 16);

  /** SETUP FRAME **/
  char buf[MAX_MSG_SIZE+32];
  int count = 0;
  // send preamble
  buf[count++] = 0xFF;
  buf[count++] = 0x00;
  buf[count++] = PREAMBLE;
  // 4 bit addresses (to & from)
  buf[count++] = ((to_addr & 0xF)<<4) | (from_addr & 0xF);
  // send length
  buf[count++] = ((msg_len*8)   ) & 0xFF;
  buf[count++] = ((msg_len*8)>>8) & 0XFF;
  memcpy(&buf[count], msg, msg_len);
  count += msg_len;
  buf[count++] = POSTAMBLE;

  // SLOW SENSING w/ randomized backoff
  /*while(readADC() > NOISE_THRESHOLD) {
    delayMicroseconds(((rand() % 100)/100.f)*MAX_WAIT_DELAY_US);
  }*/

  /** SEND FRAME **/
  send_PPM(buf, count);

  /** WAIT FOR ACK **/
  // TODO
}

float high_cutoff;
void* receive_loop(void* arg) { // PPM ONLY
  (void)arg;
  const unsigned PERIOD_LOW  = PPM_PERIOD_US-2*SAMPLE_PERIOD_US;
  const unsigned PERIOD_HIGH = PPM_PERIOD_US+2*SAMPLE_PERIOD_US;
  char buf[MAX_MSG_SIZE];

  printf("Receiver is ready.\n");
  while(1) {
    // synchronization beacon
    {
restart_receive:
      while(readADC() < high_cutoff);
      unsigned start = micros();

      // signal must stay high for one period
      while(readADC() > high_cutoff) {
        if(micros()-start > PERIOD_HIGH) goto restart_receive;
      }
      unsigned end = micros();

      // signal must stay low for one period
      while(readADC() < high_cutoff) {
        if(micros()-end > PERIOD_HIGH) goto restart_receive;
      }
      unsigned end2 = micros();

      // error check
      if(end-start <= PERIOD_LOW || end-start >= PERIOD_HIGH ||
         end2-end  <= PERIOD_LOW || end2-end  >= PERIOD_HIGH) {
        goto restart_receive;
      }
    }

    // check preamble
    {
      unsigned char received = 0;
      for(int i=0;i<8/PPM_BITS;i++) {
        received = (received<<PPM_BITS) + receivePPM();
      }
      if(received != PREAMBLE) goto restart_receive;
    }

    // get info
    char to_addr = 0, from_addr = 0;
    unsigned msg_size = 0;
    {
      unsigned addrs = 0;
      for(int i=0;i<8/PPM_BITS;i++) {
        addrs = (addrs<<PPM_BITS)+receivePPM();
      }
      to_addr = addrs>>4 & 0xF;
      from_addr = addrs & 0xF;
      for(int i=0;i<16/PPM_BITS;i++) {
        msg_size = (msg_size<<PPM_BITS)+receivePPM();
      }

      if(to_addr>=15 || from_addr>=15 || msg_size >= MAX_MSG_SIZE) {
        goto restart_receive;
      }
    }

    // now, get msg
    for(int i=0;i<msg_size;i++) {
      char curchar = 0;
      for(int i=0;i<8/PPM_BITS;i++) curchar = (curchar<<PPM_BITS)+receivePPM();
      buf[i] = curchar;
    }

    // check postamble
    {
      unsigned char received = 0;
      for(int i=0;i<8/PPM_BITS;i++) {
        received = (received<<PPM_BITS) + receivePPM();
      }
      if(received != POSTAMBLE) goto restart_receive;
    }

    // print msg
    printf("MSG RECEIVED: \"%s\"\n", buf);
  }

  return NULL;
}


#define CONFIG_VALUE 1000
int main() {
  // init rand lib
  time_t t;
  srand((unsigned)time(&t));

  // init pi
  if(wiringPiSetupGpio()<0
#ifndef SEND_ONLY
     || initADC()<0
#endif
  ){
    printf("PI setup failed!\n");
    return -1;
  }

  // init pins
  pinMode(LED_PIN, OUTPUT);


  // print configuration
  ASSERT(PPM_PERIOD_US % (1<<PPM_BITS) == 0);
  ASSERT(PPM_BITS==1 || PPM_BITS==2 || PPM_BITS==4 || PPM_BITS==8);
  printf("Config:\n");
  printf("PPM Period: %d us\n", PPM_PERIOD_US);
  printf("PPM Period: %d bits\n", PPM_BITS);
  printf("PPM slot period: %d us\n", PPM_SLOT_US);

#ifndef SEND_ONLY
  // take a few values from ADC to get mean low and stddev
  float adc_values[CONFIG_VALUE];
  float mean = 0;
  float stddev = 0;
  for(int i=0;i<CONFIG_VALUE;i++) {
    adc_values[i] = readADC();
    mean += adc_values[i];
    delayMicroseconds(1000);
  }
  mean /= CONFIG_VALUE;
  for(int i=0;i<CONFIG_VALUE;i++) {
    stddev += (adc_values[i]-mean)*(adc_values[i]-mean);
  }
  stddev = sqrt(stddev/(CONFIG_VALUE-1));

  printf("mean low value: %.3fv\n", mean);
  printf("stddev value: %.3fv\n", stddev);
  high_cutoff = mean+4*stddev;
  printf("high cutoff is therefore %.3fv\n", high_cutoff);

  // fork receiver thread
  pthread_t receiver_thread;
  if(pthread_create(&receiver_thread, NULL, receive_loop, NULL)<0) {
    printf("Failed to fork receiver thread.\n");
    return -1;
  }
#endif
  
  while(1) {
    size_t size;
    char* buf = NULL;
    printf("Type a message to send: ");
    fflush(stdout);
    if((size=getline(&buf, &size, stdin)) != -1) {
      buf[--size] = '\0';
      if(strcmp(buf, "quit")==0 || strcmp(buf, "exit")==0) {
        free(buf);
        break;
      }
      printf("Attempting to send \"%s\"...\n", buf);
      if(size > MAX_MSG_SIZE) {
        printf("FAIL: msg must be <= %d chars\n", MAX_MSG_SIZE);
      } else {
        send(buf, size, OTHER_ID, MY_ID);
        printf("Completed.\n");
      }
    }
    free(buf);
  }

#ifndef SEND_ONLY
  pthread_join(receiver_thread, NULL);
#endif

  return 0;
}
