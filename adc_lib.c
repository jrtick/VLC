#include <wiringPiSPI.h>
#include <stdio.h>

// ADC defines
#define KHZ 1000
#define SPI_CHANNEL 0
#define SPI_CLOCK (1200*KHZ)
#define ADC_RESOLUTION 10 // bits
#define V_REF 3.3 // volts
#define LOGIC_HIGH 1
#define SINGLE_MODE 1
#define MSB_FIRST 0

int initADC(){
  return wiringPiSPISetup(SPI_CHANNEL, SPI_CLOCK);
}

float readADC(void) {
  unsigned char buf[2] = {(LOGIC_HIGH<<7) | (SINGLE_MODE<<6) |
	                        (SPI_CHANNEL<<5) | (MSB_FIRST<<4), 0};
  int count = wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
  if(count != 2) {
    printf("ERROR: ADC READ FAILED.\n");
    return -2*V_REF;
  } else {
    const int value = ((buf[0]<<8) | buf[1])>>1;
    return (V_REF*1.f*value) / (1<<ADC_RESOLUTION);
  }
}

