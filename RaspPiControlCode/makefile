CC = gcc
CFLAGS = -Wall -lpthread

all: encoder controller gpio_setup step_response

encoder: encoder.c
	$(CC) $(CFLAGS) -o encoder encoder.c

controller: controller.c
	$(CC) $(CFLAGS) -o controller controller.c

gpio_setup: gpio_setup.c
	$(CC) $(CFLAGS) -o gpio_setup gpio_setup.c

step_response: step_response.c
	$(CC) $(CFLAGS) -o step_response step_response.c

clean:
	rm -f encoder controller gpio_setup step_response