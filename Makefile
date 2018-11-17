all: VLC

VLC: VLC.c
	g++ -o VLC VLC.c

clean:
	rm VLC
