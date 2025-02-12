#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <time.h>

// Global variables for frequency measurement
volatile int pulse_count = 0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// GPIO pin for encoder interrupt
#define ENCODER_GPIO_PIN 9

// Function to handle encoder interrupts
void *encoder_interrupt_thread(void *arg) {
    struct pollfd pfd;
    char buffer[2];

    // Configure GPIO for interrupt
    system("echo 9 > /sys/class/gpio/export");
    system("echo in > /sys/class/gpio/gpio9/direction");
    system("echo rising > /sys/class/gpio/gpio9/edge");

    pfd.fd = open("/sys/class/gpio/gpio9/value", O_RDONLY | O_NONBLOCK);
    pfd.events = POLLPRI;

    while (1) {
        lseek(pfd.fd, 0, SEEK_SET);
        read(pfd.fd, buffer, sizeof(buffer));
        poll(&pfd, 1, -1); // Wait for interrupt

        pthread_mutex_lock(&mutex);
        pulse_count++; // Increment pulse count
        pthread_mutex_unlock(&mutex);
    }

    close(pfd.fd);
    return NULL;
}

// Function to calculate frequency
void *frequency_calculation_thread(void *arg) {
    int sample_time_ms = 50; // Sample time in milliseconds
    int pulses_per_rotation = 500; // Encoder pulses per rotation

    while (1) {
        usleep(sample_time_ms * 1000); // Wait for sample time

        pthread_mutex_lock(&mutex);
        int current_pulse_count = pulse_count;
        pulse_count = 0; // Reset pulse count
        pthread_mutex_unlock(&mutex);

        // Calculate frequency in Hz
        float frequency = (float)current_pulse_count / (pulses_per_rotation * (sample_time_ms / 1000.0));
        printf("%.2f\n", frequency); // Output frequency to stdout
        fflush(stdout);
    }

    return NULL;
}

int main() {
    pthread_t encoder_thread, frequency_thread;

    // Create threads for encoder interrupt and frequency calculation
    pthread_create(&encoder_thread, NULL, encoder_interrupt_thread, NULL);
    pthread_create(&frequency_thread, NULL, frequency_calculation_thread, NULL);

    // Wait for threads to finish (they won't in this case)
    pthread_join(encoder_thread, NULL);
    pthread_join(frequency_thread, NULL);

    return 0;
}