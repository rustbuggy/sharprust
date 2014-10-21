CC = g++
CFLAGS  = -g -Wall -I../firmware -DDEBUG

default: main


main:  main.o mcdriver.o
	$(CC) $(CFLAGS) -o main main.o mcdriver.o

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

mcdriver.o: ../firmware/mcdriver.cpp
	$(CC) $(CFLAGS) -c ../firmware/mcdriver.cpp

clean:
	$(RM) main *.o *~
