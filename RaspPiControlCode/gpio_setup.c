#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#define GPIO_PIN 9

void setup_gpio() {
    // Export GPIO pin
    system("echo 9 > /sys/class/gpio/export");
    system("echo in > /sys/class/gpio/gpio9/direction");
    system("echo rising > /sys/class/gpio/gpio9/edge");
}

void handle_interrupt() {
    struct pollfd pfd;
    char buffer[2];

    pfd.fd = open("/sys/class/gpio/gpio9/value", O_RDONLY | O_NONBLOCK);
    pfd.events = POLLPRI;

    while (1) {
        lseek(pfd.fd, 0, SEEK_SET);
        read(pfd.fd, buffer, sizeof(buffer));
        poll(&pfd, 1, -1); // Wait for interrupt

        printf("Interrupt detected!\n");
    }

    close(pfd.fd);
}

int main() {
    setup_gpio();
    handle_interrupt();
    return 0;
}