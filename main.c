#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>


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


    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;  // 8 data bits
    options.c_iflag &= ~IGNBRK;  // Disable break processing
    options.c_lflag = 0;         // No signaling chars, no echo, no canonical processing
    options.c_oflag = 0;         // No remapping, no delays
    options.c_cc[VMIN] = 1;      // Read at least 1 character
    options.c_cc[VTIME] = 1;     // 0.1 seconds timeout

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
    options.c_cflag |= (CLOCAL | CREAD);        // Enable receiver, ignore modem control lines
    options.c_cflag &= ~(PARENB | PARODD);      // Disable parity
    options.c_cflag &= ~CSTOPB;                 // 1 stop bit
    //options.c_cflag &= ~CRTSCTS;                // Disable hardware flow control


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
    // Set up the file descriptor set
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // Set up the timeout (e.g., 2 seconds)
    struct timeval timeout;
    timeout.tv_sec = 2; // Timeout in seconds
    timeout.tv_usec = 0;

    // Wait for the serial port to become ready
    int result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0) {
        perror("select() failed");
        return -1;
    } else if (result == 0) {
        fprintf(stderr, "Timeout waiting for data on the serial port.\n");
        return -1;
    }

    // Read data from the serial port

    int bytes_read = read(fd, buffer, buffer_size - 1);
    if (bytes_read < 0) {
        perror("Failed to read from serial port");
        return -1;
    }

    // Null-terminate the string
    buffer[bytes_read] = '\0';
    return bytes_read;
}


