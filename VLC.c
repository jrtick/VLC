// standard includes
#include <stdio.h>
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

// hardware defines
#define LED_PIN 25 // GPIO
#define PD_RISE_US 80
#define PD_FALL_US 80
#define SAMPLE_PERIOD_US 25 // 40kHz
#define PPM_BITS 1 //  manchester encoding
#define PPM_SLOT_COUNT (1<<(PPM_BITS))
#define PPM_SLOT_US (SAMPLE_PERIOD_US*100) // should probs never go below 10
#define PPM_PERIOD_US (PPM_SLOT_US*PPM_SLOT_COUNT)

// Protocol defines
// preamble and postamble send least significant bit first
#define PREAMBLE  0b01010101
#define POSTAMBLE 0b00100100
#define MAX_MSG_SIZE 60 // bytes
#define PACKET_PERIOD_US ((8/PPM_BITS)*PPM_PERIOD_US*(MAX_MSG_SIZE+4))

#define BEACON_PERIOD_US (4*PPM_PERIOD_US)
#define SLOW_SENSING_PERIOD_US BEACON_PERIOD_US
#define RANDOM_BACKOFF_LOW_US PACKET_PERIOD_US
#define RANDOM_BACKOFF_RANGE_US (4*PACKET_PERIOD_US)

// globals
static float high_cutoff;
static volatile bool SENDING = false;
static volatile bool end_of_program = false;

inline int min(const int x, const int y) {
  return x<y? x : y;
}

static unsigned recorded_time;
static unsigned period_start;
void reset_led_tracking(void) {
  period_start = micros();
  recorded_time = 0;
}

void set_led(const bool value, const unsigned duration) {
  const unsigned new_contract_start = micros();
  static bool led = false;
  static unsigned contract_start=(unsigned)-1;
  static int remaining_time=0;
  if(value != led) {
    // finish contract
    recorded_time += remaining_time;
    //if(contract_start != (unsigned)-1) remaining_time -= (micros()-contract_start);
    /*if(remaining_time > 0) {
      // do a real v hypothetical correction
      const int difference = (micros()-period_start) - recorded_time;
      if(difference>2) {
        const int change = min(min(difference-1, remaining_time), 25);
        recorded_time += change;
        remaining_time -= change;
      } //else if(difference < 0) delayMicroseconds(min(-difference, 25));
    }*/

    if(remaining_time>0) {
      const int offset = led? PD_FALL_US : PD_RISE_US;
      if(remaining_time > offset) {
        delayMicroseconds(remaining_time-offset);
        remaining_time = offset;
      }
    } else remaining_time = 0;
    digitalWrite(LED_PIN, value);
    contract_start = new_contract_start;
    led = value;
  }
  remaining_time += duration;
}

/** this is Manchester encoding if PPM_BITS==1 */
int send_PPM(const char* buf, const int byte_count) {
  { // slow sensing -- check if someone else is currently sending
    unsigned start;
    int contended = -1;
slow_sensing:
    contended++;
    if(contended==1) {
      printf("CONTENDED...");
      fflush(stdout);
    } else if(contended>1) {
      printf(".");
      fflush(stdout);
    }
    start = micros();
    while(micros()-start < SLOW_SENSING_PERIOD_US) {
      if(readADC() > high_cutoff) {
        // random backoff if collision detected
        delayMicroseconds(RANDOM_BACKOFF_LOW_US+(rand() % RANDOM_BACKOFF_RANGE_US));
        goto slow_sensing;
      }
    }
    if(contended>0) printf("GOING\n");
  }

  SENDING = true;

  // send beacon
  reset_led_tracking();
  set_led(true , BEACON_PERIOD_US/2);
  set_led(false, BEACON_PERIOD_US/2);

  // for each byte
  for(int i=0;i<byte_count;i++) {
    const char cur_char = buf[i];
    // for each ppm symbol in current byte
    for(int j=0;j<8;j+=PPM_BITS) {
      const char cur_val = (cur_char>>j) & ((1<<PPM_BITS)-1);

      if(cur_val != 0) set_led(false, PPM_SLOT_US*cur_val);
      set_led(true, PPM_SLOT_US);
      if(cur_val != PPM_SLOT_COUNT-1) {
        set_led(false, ((PPM_SLOT_COUNT-1)-cur_val)*PPM_SLOT_US);
      }
    }
  }
  set_led(false, 0);
  SENDING = false;
  return 0;
}



static volatile int count_buf_index = 0;
static int count_buf[1000];
// receives one period of PPM
inline char receivePPM() {
  int on_slot = 0;
  int max_count = 0;
  // vote on which slot is high based on which has the most "high" values
  for(int cur_slot=0; cur_slot<PPM_SLOT_COUNT; cur_slot++) {
    const unsigned start = micros();
    int count = 0;
    while((micros()-start) <= PPM_SLOT_US-5*SAMPLE_PERIOD_US) {
      const float val = readADC();
      if(val < 0) printf("ERR\n");
      if(val >= high_cutoff) count++;
    }
    if(count>max_count) {
      max_count = count;
      on_slot = cur_slot;
    }
    count_buf[count_buf_index++] = count;

    // account for skew by finishing a little early and waiting for next slot
    delayMicroseconds(PPM_SLOT_US-(micros()-start));
  }

  return on_slot;
}

int send(const char* msg, const int msg_len,
         const char to_addr, const char from_addr) {
  ASSERT(msg_len < MAX_MSG_SIZE); // TODO: split longer messages up into several packets
  ASSERT(to_addr < 16);
  ASSERT(from_addr < 16);

  /** SETUP FRAME **/
  char buf[MAX_MSG_SIZE+4];
  int count = 0;

  buf[count++] = PREAMBLE;
  buf[count++] = ((to_addr & 0xF)<<4) | (from_addr & 0xF);
  buf[count++] = (char) msg_len;
  memcpy(&buf[count], msg, msg_len);
  count += msg_len;
  buf[count++] = POSTAMBLE;

  /** SEND FRAME, print it for debugging **/
  for(int i=0;i<count;i++) {
    for(int j=0;j<8;j++) printf("%d", (buf[i] >> j) & 1);
    printf(".");
  }
  printf("\n");
  send_PPM(buf, count);

  // TODO : look for ACK
  return 0;
}

void* receive_loop(void* const arg) {
  char buf[MAX_MSG_SIZE+1];

  while(!end_of_program) {
restart_receive:
    if(count_buf_index) {
      for(int i=0;i<count_buf_index;i+=2) {
        printf("%d vs %d\n", count_buf[i], count_buf[i+1]);
      }
      count_buf_index = 0;
    }

    /** wait for a signal **/
    while(readADC() < high_cutoff) {
      if(end_of_program) return arg;
    }
    // if it's our signal, do nothing.
    if(SENDING) {
      while(SENDING);
      continue;
    }
 

    /** synchronization beacon **/
    {
      // signal must stay high for half of beacon
      unsigned dur, start = micros();
      while((dur = micros()-start) < BEACON_PERIOD_US/2-5*SAMPLE_PERIOD_US) {
        float val = 0;
        for(int i=0;i<4;i++) val+=readADC();
        if(val/4 < high_cutoff) {
          if(dur>100) printf("failed beacon, only %dus\n", dur);
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
          printf("Failed off beacon, only %dus\n", dur);
          goto restart_receive;
        }
      }
      delayMicroseconds(BEACON_PERIOD_US/2-dur);
    }

    /** check preamble **/
    {
      unsigned char received = 0;
      for(int i=0; i<8; i+=PPM_BITS) received |= receivePPM()<<i;
      if(received != PREAMBLE) {
        printf("Failed PREAMBLE (detected 0x%x)\n", received);
        goto restart_receive;
      }
    }

    /** get packet info **/
    char to_addr, from_addr;
    unsigned msg_size = 0;
    {
      unsigned addrs = 0;
      for(int i=0;i<8;i+=PPM_BITS) addrs |= receivePPM()<<i;
      to_addr = (addrs & 0xF0) >> 4;
      from_addr = addrs & 0x0F;
      // TODO: ignore if not our address??
      for(int i=0;i<8;i+=PPM_BITS) msg_size |= receivePPM()<<i;

      if(msg_size >= MAX_MSG_SIZE) {
        printf("invalid params (to=%d, from=%d, msg_size=%d\n",
               to_addr, from_addr, msg_size);
        goto restart_receive;
      }
    }

    /** now, get actual message **/
    for(int i=0;i<msg_size;i++) {
      char* curchar = &buf[i];
      *curchar = 0;
      for(int i=0;i<8;i+=PPM_BITS) *curchar |= receivePPM()<<i;
    }
    buf[msg_size] = '\0';

    /** check postamble **/
    {
      unsigned char received = 0;
      for(int i=0;i<8;i+=PPM_BITS) received |= receivePPM()<<i;
      if(received != POSTAMBLE) {
        printf("failed POSTAMBLE (detected 0x%x)\n", received);
        printf("to=%d,from=%d,msglen=%d\n", to_addr, from_addr, msg_size);
        printf("message would've been: ");
        for(int i=0;i<msg_size;i++) printf("0x%x ", buf[i]);
        printf("\n", buf);
        goto restart_receive;
      }
    }

    /** acknowledge successful receipt **/
    delayMicroseconds(5*SAMPLE_PERIOD_US);
    send("ack", 3, from_addr, to_addr);

    // print received msg
    printf("(%d -> %d) MSG RECEIVED (%d): \"%s\"\n",
           from_addr, to_addr, msg_size, buf);
  }

  return arg;
}

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
  ASSERT(PPM_SLOT_US > 2*(PD_FALL_US+PD_RISE_US));
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
  
  while(1) {
    size_t size = 0;
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
  end_of_program = true;
  pthread_join(receiver_thread, NULL);
#endif

  return 0;
}
