/** This is the main file for our VLC communication via
 * photodiodes and LEDs. The other files in this folder are for
 * debug purposes.
 *
 * The main thread is used for transmitting. It first forks a thread
 * which is used to receive incoming messages.
 *
 * We use the WiringPi library to control the Pi's GPIO pins.
 * We used pthreads for threading.
 *
 * We communicate between threads via volatile bools that act as flags.
 */

// standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
// important includes
#include <wiringPi.h>
#include <pthread.h>
#include "adc_lib.h"
#include "ids.h"

// random defines
#define STR(x) #x
#define ASSERT(cond) \
  if(!(cond)) { \
    printf("assertion '%s' failed at line %d\n", STR(cond), __LINE__); \
    exit(-1); \
  }
//#define DEBUG_INFO

// hardware defines
#define LED_PIN 25 // GPIO pin number for the LED
#define SAMPLE_PERIOD_US 25 // ADC is approx ~40kHz

// Protocol defines
/* Modulation defines */
#define PPM_BITS 1 // equivalent to Manchester encoding
#define PPM_SLOT_COUNT (1<<(PPM_BITS))
#define PPM_SLOT_US (SAMPLE_PERIOD_US*20) // We've tested as low as 5
#define PPM_PERIOD_US (PPM_SLOT_US*PPM_SLOT_COUNT)

/* packet frame defines */
/***We send/receive least significant bit first***/
#define PREAMBLE  0b01010101
#define POSTAMBLE 0b00100100
#define MAX_MSG_SIZE 60 // bytes
#define PACKET_PERIOD_US ((8/PPM_BITS)*PPM_PERIOD_US*(MAX_MSG_SIZE+4))

/* Higher level defines */
#define BROADCAST_ADDR 0xF // this is broadcast address, all others must be below

#define BEACON_PERIOD_US (4*PPM_PERIOD_US)
#define SLOW_SENSING_PERIOD_US BEACON_PERIOD_US
#define RANDOM_BACKOFF_LOW_US PACKET_PERIOD_US
#define RANDOM_BACKOFF_RANGE_US (4*PACKET_PERIOD_US)

// globals
static float high_cutoff; // what photodiode voltage level constitutes "HIGH"
static volatile bool SENDING = false; // whether we are currently transmitting
static volatile bool end_of_program = false; // whether program should end
static volatile int ack_received; // bitmask of which addresses acked last msg

/** Given a list of bits, send them in a PPM fashion */
int send_PPM(const char* buf, const int byte_count) {
  // Preprocess step: pre-record PPM modulation of enter signal
  bool signal[PPM_SLOT_COUNT*(8/PPM_BITS)*(MAX_MSG_SIZE+4)] = {0};
  for(int i=0;i<byte_count;i++) {
    const char cur_char = buf[i];
    for(int j=0;j<8/PPM_BITS;j++) {
      const char cur_val = (cur_char>>(j*PPM_BITS)) & ((1<<PPM_BITS)-1);
      signal[PPM_SLOT_COUNT*(8/PPM_BITS)*i+j*PPM_SLOT_COUNT+cur_val] = 1;
    }
  }

#ifndef SEND_ONLY
  { // slow sensing -- check if someone else is currently sending
    unsigned start;
    int contended = -1;
slow_sensing:
    contended++;
#ifdef DEBUG_INFO
    if(contended>1) {
      printf(".");
      fflush(stdout);
    }
#endif
    start = micros();
    // read the ADC for a little bit. Claim contention if you sense HIGH
    while(micros()-start < SLOW_SENSING_PERIOD_US) {
      float val = 0;
      for(int i=0;i<4;i++) val += readADC();
      if(val/4 > high_cutoff) {
#ifdef DEBUG_INFO
        if(contended==0) {
          printf("CONTENDED...");
          fflush(stdout);
        }
#endif

        // random backoff if collision detected
        delayMicroseconds(RANDOM_BACKOFF_LOW_US+(rand() % RANDOM_BACKOFF_RANGE_US));
        goto slow_sensing;
      }
    }
#ifdef DEBUG_INFO
    if(contended>0) printf("GOING\n");
#endif
  }
#endif

  SENDING = true;

  // send alignment beacon
  digitalWrite(LED_PIN, 1);
  delayMicroseconds(BEACON_PERIOD_US/2);
  digitalWrite(LED_PIN, 0);
  delayMicroseconds(BEACON_PERIOD_US/2);

  // send actual packet
  const unsigned packet_start = micros();
  const unsigned packet_duration = (byte_count)*(8/PPM_BITS)*PPM_PERIOD_US;
  unsigned duration;
  bool led_value = 0;
  // essentially query the "timeline" we setup as "signal" to see whether
  // the signal should be HIGH or LOW at a given moment
  while((duration=micros()-packet_start) < packet_duration) {
    const unsigned next_led_val = signal[duration/PPM_SLOT_US];
    if(led_value != next_led_val) {
      digitalWrite(LED_PIN, next_led_val);
      led_value = next_led_val;
    }
  }
  digitalWrite(LED_PIN, 0);

  SENDING = false;
  return 0;
}



// receives one byte from PPM 
inline char receivePPM() {
  int buf[(8/PPM_BITS)*PPM_SLOT_COUNT]={0};
  const unsigned start = micros();
  unsigned duration;
  // for the duration of receiving the byte, just write it to a buffer
  while((duration=micros()-start) < (8/PPM_BITS)*PPM_PERIOD_US-3*SAMPLE_PERIOD_US) {
    if(readADC() > high_cutoff) {
      buf[duration/PPM_SLOT_US]++;
    }
  }

  // once you have the buffer, decode it really fast.
  // We ignore the last 75us of the byte transmission because we should already
  // roughly know what the last bit is by then and this allows more than
  // enough time to decode in realtime
  char received = 0;
  for(int i=0;i<8/PPM_BITS;i++) {
    int on_slot = 0;
    int max_count = 0;
    for(int j=0;j<PPM_SLOT_COUNT;j++) {
      if(buf[i*PPM_SLOT_COUNT+j] > max_count) {
        max_count = buf[i*PPM_SLOT_COUNT+j];
        on_slot = j;
      }
    }
    received |= on_slot << (i*PPM_BITS);
  }
  // wait for byte transmission to be over
  delayMicroseconds((8/PPM_BITS)*PPM_PERIOD_US - (micros()-start));
  return received;
}

int send(const char* msg, const int msg_len,
         const char to_addr, const char from_addr, const bool ack_requested) {
  ASSERT(msg_len < MAX_MSG_SIZE);
  ASSERT(to_addr < 16);
  ASSERT(from_addr < 16);

  /** SETUP Packet FRAME **/
  char buf[MAX_MSG_SIZE+4];
  int count = 0;

  buf[count++] = PREAMBLE;
  buf[count++] = ((to_addr & 0xF)<<4) | (from_addr & 0xF);
  buf[count++] = (ack_requested << 7) | msg_len;

  // copy message here
  memcpy(&buf[count], msg, msg_len);
  count += msg_len;

  buf[count++] = POSTAMBLE;

  /** SEND FRAME, print it first for debugging purposes **/
#ifdef DEBUG_INFO
  for(int i=0;i<count;i++) {
    for(int j=0;j<8;j++) printf("%d", (buf[i] >> j) & 1);
    printf(".");
  }
  printf("\n");
#endif

  ack_received = 0;
  send_PPM(buf, count);

  // Wait for ACK with a 2 Packet period max timeout
  // If broadcasting, just wait 20 packet periods to cautiously account
  // for many possible collisions of receivers responding
  if(ack_requested) {
    const unsigned start = micros();
    if(to_addr == BROADCAST_ADDR) {
      while((micros()-start)<20*PACKET_PERIOD_US);
    } else {
      while((micros()-start)<2*PACKET_PERIOD_US && ack_received == 0);
    }
    return ack_received;
  } else return 0;
}

/** THIS IS OUR RECEIVER THREAD. It runs forever looking for messages */
void* receive_loop(void* const arg) {
  char buf[MAX_MSG_SIZE+1];

  while(!end_of_program) {
restart_receive:
    delayMicroseconds(rand() % PPM_SLOT_US);
    /** wait for a HIGH signal **/
    while(readADC() < high_cutoff) {
      if(end_of_program) return arg;
    }
    // if it's our own signal, do nothing.
    if(SENDING) {
      while(SENDING);
      continue;
    }
 
    /** alignment beacon **/
    {
      // signal must stay high for half of beacon
      unsigned dur, start = micros();
      while((dur = micros()-start) < BEACON_PERIOD_US/2-5*SAMPLE_PERIOD_US) {
        float val = 0;
        for(int i=0;i<4;i++) val+=readADC();
        if(val/4 < high_cutoff) {
          //if(dur>100) printf("failed beacon, only %dus\n", dur);
          goto restart_receive;
        }
      }
      delayMicroseconds(BEACON_PERIOD_US/2-dur);

      // signal must stay low for half of beacon
      start = micros();
      delayMicroseconds(2*SAMPLE_PERIOD_US); //ignore first few samples
      while((dur = micros()-start) < BEACON_PERIOD_US/2-5*SAMPLE_PERIOD_US) {
        float val = 0;
        for(int i=0;i<4;i++) val+=readADC();
        if(val/4 > high_cutoff) {
          //printf("Failed off beacon, only %dus\n", dur);
          goto restart_receive;
        }
      }
      delayMicroseconds(BEACON_PERIOD_US/2-dur);
    }

    // Hidden terminal avoidance -- set your own LED high so others
    // know that you are busy and can't receive right now
    digitalWrite(LED_PIN, 1);

    /** check preamble of incoming message **/
    {
      const char received = receivePPM();
      if(received != PREAMBLE) {
        digitalWrite(LED_PIN, 0);
        printf("Failed PREAMBLE (detected 0x%x)\n", received);
        goto restart_receive;
      }
    }

    /** get packet info **/
    char to_addr, from_addr;
    unsigned msg_size = 0;
    bool ack_requested;
    {
      const unsigned addrs = receivePPM();
      to_addr = (addrs & 0xF0) >> 4;
      from_addr = addrs & 0x0F;
      msg_size = receivePPM();
      ack_requested = (msg_size>>7) & 1;
      msg_size = msg_size & ~(1<<7);

      if(msg_size >= MAX_MSG_SIZE) {
        digitalWrite(LED_PIN, 0);
#ifdef DEBUG_INFO
        printf("invalid params (to=%d, from=%d, msg_size=%d\n",
               to_addr, from_addr, msg_size);
#endif
        goto restart_receive;
      }
    }

    /** now, receive actual message **/
    for(int i=0;i<msg_size;i++) {
      buf[i] = receivePPM();
    }
    buf[msg_size] = '\0';

    /** check postamble, effectively for error detection **/
    {
      const char received = receivePPM();
      digitalWrite(LED_PIN, 0);
      if(received != POSTAMBLE) {
#ifdef DEBUG_INFO
        printf("failed POSTAMBLE (detected 0x%x)\n", received);
        printf("to=%d,from=%d,msglen=%d\n", to_addr, from_addr, msg_size);
        printf("message would've been: ");
        for(int i=0;i<msg_size;i++) printf("0x%x ", buf[i]);
        printf("\n", buf);
#endif
        goto restart_receive;
      }
    }

    /** acknowledge successful receipt only if we are intended recipient **/
    printf("SNOOP: %d->%d says \"%s\"\n", from_addr, to_addr, buf);
    if(from_addr != MY_ID && (to_addr==MY_ID || to_addr==BROADCAST_ADDR)) {
      if(strcmp("ack", buf) == 0) {
        ack_received |= (1<<from_addr);
      } else {
        if(ack_requested) {
          delayMicroseconds(2*SAMPLE_PERIOD_US);
          send("ack", 3, from_addr, MY_ID, false);
        }

        // print received msg
        printf("(%d -> %d) MSG RECEIVED (%d): \"%s\"\n", from_addr, to_addr,
               msg_size, buf);
      }
    }
  }

  return arg;
}

int main() {
  // init rand lib
  time_t t;
  srand((unsigned)time(&t));

  // init Pi
  if(wiringPiSetupGpio()<0
#ifndef SEND_ONLY
     || initADC()<0
#endif
  ){
    printf("PI setup failed!\n");
    return -1;
  }

  // init LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);

  // print Configuration of network
  ASSERT(PPM_PERIOD_US % (1<<PPM_BITS) == 0);
  ASSERT(PPM_BITS==1 || PPM_BITS==2 || PPM_BITS==4 || PPM_BITS==8);
  printf("Config:\n");
  printf("Beacon Period: %d us\n", BEACON_PERIOD_US);
  printf("PPM Period: %d us\n", PPM_PERIOD_US);
  printf("PPM %d bits\n", PPM_BITS);
  printf("Packet max period: %d us\n", PACKET_PERIOD_US);

#ifndef SEND_ONLY
  // sample ADC to get mean and stddev of "OFF" voltage
  float mean = 0;
  float stddev = 0;
  int count = 0;
  const unsigned start = micros();
  while(micros()-start<2*1e6) { // take low readings for ~2s
    count++;
    const float val = readADC();
    mean += val;
    stddev += val*val;
    delayMicroseconds(1e3);
  }
  mean /= count;
  stddev = sqrt((stddev-count*mean*mean)/(count-1));

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
 
  /* This is code to send messages directly from the command line
  while(1) {
    size_t size = 0;
    char* buf = NULL;

    printf("Type an address to send to: ");fflush(stdout);
    if((size=getline(&buf, &size, stdin)) == -1) {
      free(buf);
      continue;
    }
    buf[--size] = '\0';
    if(strcmp(buf, "quit")==0 || strcmp(buf, "exit")==0) {
      free(buf);
      break;
    }
    const int send_addr = atoi(buf);
    if((buf[0]<'0' || buf[0]>'9') || send_addr<0 || send_addr > BROADCAST_ADDR) {
      printf("invalid address. Please try again.\n");
      free(buf);
      continue;
    }

    free(buf);
    buf = NULL;
    size = 0; 
    printf("Type a message to send: ");fflush(stdout);
    if((size=getline(&buf, &size, stdin)) != -1) {
      buf[--size] = '\0';
      printf("Attempting to send \"%s\" (%d->%d)...\n", buf, MY_ID, send_addr);
      if(size > MAX_MSG_SIZE) {
        printf("FAIL: msg must be <= %d chars\n", MAX_MSG_SIZE);
      } else {
        const int result = send(buf, size, send_addr, MY_ID, true);
        if(result == 0) printf("NO ACK\n");
        else {
          for(int i=0;i<BROADCAST_ADDR;i++) {
            if(result & (1<<i)) printf("We got an ack from %d\n", i);
          }
        }
      }
    }
    free(buf);
  }*/

  { // this is a hardcoded test of our network, sending the same message 100 times
    int acks = 0;
    const unsigned start = millis();
    for(int i=0;i<100;i++) {
      //const int result = send("hello", 5, 3, MY_ID, true);
      const int result = send("12345678901234567890123456789012345678901234567890123456789", 59, 3, MY_ID, true);
      printf("result=%d\n", result);
      if(result & (1<<3)) {
        acks++;
        printf("ack\n");
      } else {
        delayMicroseconds(rand() % PPM_SLOT_US); // if lost packet, backoff a random amount. It may help
      }
    }
    const unsigned end = millis();
    printf("We got %d of %d acks in %ums\n", acks, 100, (end-start));
  }

#ifndef SEND_ONLY
  end_of_program = true;
  pthread_join(receiver_thread, NULL);
#endif

  return 0;
}
