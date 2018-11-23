all: VLC ADC pin

VLC: VLC.c adc_lib.c adc_lib.h
	g++ -o VLC VLC.c adc_lib.c -lwiringPi -lpthread
ADC: ADC.c adc_lib.c adc_lib.h
	g++ -o ADC ADC.c adc_lib.c -lwiringPi
pin: pin.c
	g++ -o pin pin.c -lwiringPi

clean:
	rm VLC ADC pin
