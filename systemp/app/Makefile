CC = gcc
CFLAGS = -Wall
INCLUDES = -I../device_driver

all: tx_app rx_app

tx_app: tx_app.c
\t$(CC) $(CFLAGS) tx_app.c -o tx_app

rx_app: rx_app.c
\t$(CC) $(CFLAGS) rx_app.c -o rx_app

clean:
\trm -f tx_app rx_app
