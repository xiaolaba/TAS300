// command line to build, 
// gcc -Wall -O2 tas300.c -o tas300.exe
//


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE_NUM    9600
#define BUFFER_SIZE     256
#define TIMEOUT_SEC     3

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    typedef HANDLE SerialPortHandle;
    #define INVALID_PORT_HANDLE INVALID_HANDLE_VALUE
    #define DEFAULT_PORT "\\\\.\\COM8"
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <termios.h>
    #include <errno.h>
    #include <sys/select.h>
    typedef int SerialPortHandle;
    #define INVALID_PORT_HANDLE -1
    #define DEFAULT_PORT "/dev/ttyS8"
#endif

// Open and configure serial port for Windows or Linux
SerialPortHandle open_and_config_serial(const char *port_name) {
#if defined(_WIN32) || defined(_WIN64)
    HANDLE hCom = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hCom == INVALID_HANDLE_VALUE) {
        printf("[Win32 Error] Cannot open %s (Code: %lu)\n", port_name, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hCom, &dcb);

    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;

    // Hardware Flow Control (RTS/CTS)
    dcb.fOutxCtsFlow = TRUE;
    dcb.fRtsControl  = RTS_CONTROL_HANDSHAKE;
    dcb.fBinary      = TRUE;

    if (!SetCommState(hCom, &dcb)) {
        printf("[Win32 Error] SetCommState failed (Code: %lu)\n", GetLastError());
        CloseHandle(hCom);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = TIMEOUT_SEC * 1000;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 1000;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hCom, &timeouts);

    return hCom;
#else
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("[Linux Error] open()");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB;        // No parity
    options.c_cflag &= ~CSTOPB;        // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8 data bits
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag |= CRTSCTS;        // RTS/CTS Hardware Flow Control

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[Linux Error] tcsetattr()");
        close(fd);
        return -1;
    }

    return fd;
#endif
}

// Send command frame and receive response line
int send_tas300_command(SerialPortHandle handle, const char *cmd, char *response_out, size_t max_len) {
    char tx_buffer[BUFFER_SIZE];
    snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", cmd);

#if defined(_WIN32) || defined(_WIN64)
    DWORD bytes_written = 0;
    DWORD bytes_read = 0;

    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    if (!WriteFile(handle, tx_buffer, (DWORD)strlen(tx_buffer), &bytes_written, NULL)) {
        printf("[Win32 Error] WriteFile failed (Code: %lu)\n", GetLastError());
        return -1;
    }
    printf("--> Sent: %s", tx_buffer);

    memset(response_out, 0, max_len);
    DWORD total_bytes = 0;
    char c = 0;

    while (total_bytes < (max_len - 1)) {
        if (ReadFile(handle, &c, 1, &bytes_read, NULL) && bytes_read > 0) {
            response_out[total_bytes++] = c;
            if (c == '\n') break;
        } else {
            break; // Timeout or read complete
        }
    }

    if (total_bytes == 0) {
        printf("<-- Error: Read timeout from TAS300 controller.\n");
        return -1;
    }
    printf("<-- Received: %s", response_out);
    return 0;
#else
    tcflush(handle, TCIOFLUSH);

    ssize_t written = write(handle, tx_buffer, strlen(tx_buffer));
    if (written < 0) {
        perror("[Linux Error] write()");
        return -1;
    }
    printf("--> Sent: %s", tx_buffer);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(handle, &readfds);

    struct timeval tv;
    tv.tv_sec = TIMEOUT_SEC;
    tv.tv_usec = 0;

    if (select(handle + 1, &readfds, NULL, NULL, &tv) <= 0) {
        fprintf(stderr, "<-- Error: Read timeout from TAS300 controller.\n");
        return -1;
    }

    memset(response_out, 0, max_len);
    ssize_t total_read = 0;
    while (total_read < (ssize_t)(max_len - 1)) {
        char c;
        if (read(handle, &c, 1) > 0) {
            response_out[total_read++] = c;
            if (c == '\n') break;
        } else {
            break;
        }
    }
    printf("<-- Received: %s", response_out);
    return 0;
#endif
}

// Close port
void close_serial_port(SerialPortHandle handle) {
#if defined(_WIN32) || defined(_WIN64)
    if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
#else
    if (handle >= 0) close(handle);
#endif
}

int main(int argc, char *argv[]) {
    const char *port_name = (argc > 1) ? argv[1] : DEFAULT_PORT;

    printf("Opening TAS300 connection on %s...\n", port_name);

    SerialPortHandle handle = open_and_config_serial(port_name);
    if (handle == INVALID_PORT_HANDLE) {
        return EXIT_FAILURE;
    }

    char rx_buf[BUFFER_SIZE];

    printf("\n--- Query Firmware Version ---\n");
    send_tas300_command(handle, "VER", rx_buf, sizeof(rx_buf));

    printf("\n--- Reading Sensor Bitmap ---\n");
    send_tas300_command(handle, "GETS", rx_buf, sizeof(rx_buf));

    close_serial_port(handle);
    printf("\nPort closed successfully.\n");
    return EXIT_SUCCESS;
}