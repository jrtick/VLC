
#define PREAMBLE 0b10101010
#define PREAMBLE_LENGTH 8
#define POSTAMBLE 0b00100100 // TODO
#define POSTAMBLE_LENTH 8
#define MAX_BUF_SIZE 128
#define MAX_MSG_SIZE 512
#define PERIOD 100 // MUST BE FACTOR OF 2
#define NOISE_THRESHOLD 1.f
#define MAX_WAIT_DELAY_US MAX_MSG_SIZE*PERIOD

#define ASSERT (cond) printf("assertion '%s' failed at line %d\n", #cond, __LINE__);

float readADC(void) {
  return 0.f; // TODO
}
inline void writeLED(const bool value, const unsigned microseconds) {
  return;
}
inline void wait(const unsigned microseconds) {
  return;
}

int send_OOK(const char* buf, const int len) {
  for(int i=0;i<len;i++) {
    writeLED(buf[i], PERIOD);
  }
  // TODO: FAST SENSING
  return 0;
}

int send_ME(const char* buf, const int len) {
  for(int i=0;i<len;i++) {
    if(buf[i]) {
      writeLED(0, PERIOD/2);
      writeLED(1, PERIOD/2);
    } else {
      writeLED(1, PERIOD/2);
      writeLED(0, PERIOD/2);
    }
  }
  return 0;
}


int receive_OOK(char* buf, const int max_bytes) {
  
}

int send(const char* msg, const int msg_len,
         const char to_addr, const char from_addr) {
  ASSERT(msg_len < MAX_MSG_SIZE);
  ASSERT(to_addr < 16);
  ASSERT(from_addr < 16);

  /** SETUP FRAME **/
  char buf[MAX_BUF_SIZE];
  int offset = 0;
  // send preamble
  buf[offset++] = PREAMBLE;
  // 4 bit addresses (to & from)
  buf[offset++] = ((to_addr & 0xF)<<4) | (from_addr & 0xF);
  // send length
  buf[offset++] = ((msg_len*8)   ) & 0xFF;
  buf[offset++] = ((msg_len*8)>>8) & 0XFF;
  // send msg
  for(int i=0;i<msg_len;i++) {
    buf[offset++] = msg[i];
  }

  /** SLOW SENSING **/
  while(readADC() > NOISE_THRESHOLD);
  /* // TODO: randomized backoff
  while(readADC() > NOISE_THRESHOLD) {
    wait(((rand() % 100)/100.f)*MAX_WAIT_DELAY_US);
  }
  */

  /** SEND FRAME **/
  send_OOK(buf, offset);

  /** WAIT FOR ACK **/
  
}

int main() {
  time_t t;
  srand((unsigned)time(&t));

  // init pi
}
