CROSS_COMPILE ?=

CC	:= $(CROSS_COMPILE)gcc
CFLAGS	?= -O0 -W -Wall -g
LDFLAGS	?=
LIBS	:= -lrt

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: yavta

yavta: yavta.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o
	-rm -f yavta

