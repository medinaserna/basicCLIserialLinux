#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>


int open_serial_port(const char *device);
int write_to_serial(int fd, const char *data);
int read_from_serial(int fd, char *buffer, size_t buffer_size); 



int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <serial_port> <string_to_send>\n", argv[0]);
        return 1;
    }

    const char *serial_port = argv[1];
    const char *string_to_send = argv[2];

    // Open and configure the serial port
    int fd = open_serial_port(serial_port);
    if (fd == -1) {
        return 1;
    }

    // Write data to the serial port
    if (write_to_serial(fd, string_to_send) == -1) {
        close(fd);
        return 1;
    }

    // Read response from the serial port
    char response[256];
    if (read_from_serial(fd, response, sizeof(response)) == -1) {
        close(fd);
        return 1;
    }

    // Display the response
    printf("Response from serial port: %s\n", response);

    // Close the serial port
    close(fd);
    return 0;
}

int open_serial_port(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Configure serial settings
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, local mode
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Clear current character size mask
    options.c_cflag |= CS8;              // 8 data bits

    // Apply settings
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}



int write_to_serial(int fd, const char *data) {
    int bytes_written = write(fd, data, strlen(data));
    if (bytes_written < 0) {
        perror("Failed to write to serial port");
        return -1;
    }
    return bytes_written;
}


int read_from_serial(int fd, char *buffer, size_t buffer_size) {
    int bytes_read = read(fd, buffer, buffer_size - 1);
    if (bytes_read < 0) {
        perror("Failed to read from serial port");
        return -1;
    }

    // Null-terminate the string
    buffer[bytes_read] = '\0';
    return bytes_read;
}

