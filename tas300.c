// command line to build, 
// gcc -Wall -O2 tas300.c -o tas300.exe
//

/*
commands for,
mechanical motion 
door latch operations
wafer mapping
sensor status decoding 
error resets
firmware identification
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#define BAUDRATE_NUM    19200
#define BUFFER_SIZE     256
#define TIMEOUT_SEC     10  // Timeout for long mechanical movements (e.g., LOAD/UNLD)

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

// Decoded TAS300 Sensor State Structure
typedef struct {
    uint16_t raw_hex_value;
    int info_pad_a;       // Bit 0
    int info_pad_b;       // Bit 1
    int info_pad_c;       // Bit 2
    int info_pad_d;       // Bit 3
    int placement_left;   // Bit 4
    int placement_right;  // Bit 5
    int presence_front;   // Bit 6
    int presence_rear;    // Bit 7
    int foup_clamped;     // Bit 8
    int door_opened;      // Bit 9
    int z_stage_down;     // Bit 10
    int emo_active;       // Bit 11
    int main_air_ok;      // Bit 12
} TAS300_Sensors;

// Low-Level Serial Management configured for 19200 Baud
SerialPortHandle open_and_config_serial(const char *port_name) {
#if defined(_WIN32) || defined(_WIN64)
    HANDLE hCom = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hCom == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hCom, &dcb);

    // Set Baud Rate to 19200
    dcb.BaudRate = CBR_19200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    
    // Hardware Flow Control (RTS/CTS)
    dcb.fOutxCtsFlow = TRUE;
    dcb.fRtsControl  = RTS_CONTROL_HANDSHAKE;
    dcb.fBinary      = TRUE;

    if (!SetCommState(hCom, &dcb)) {
        CloseHandle(hCom);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 20;
    timeouts.ReadTotalTimeoutConstant    = TIMEOUT_SEC * 1000;
    timeouts.ReadTotalTimeoutMultiplier  = 5;
    timeouts.WriteTotalTimeoutConstant   = 1000;
    timeouts.WriteTotalTimeoutMultiplier = 5;
    SetCommTimeouts(hCom, &timeouts);

    return hCom;
#else
    int fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // Set Baud Rate to 19200
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag |= CRTSCTS; // Hardware Flow Control

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        close(fd);
        return -1;
    }

    return fd;
#endif
}

void close_serial_port(SerialPortHandle handle) {
#if defined(_WIN32) || defined(_WIN64)
    if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
#else
    if (handle >= 0) close(handle);
#endif
}




// Send Command Frame & Receive Line
int send_tas300_command(SerialPortHandle handle, const char *cmd, char *response_out, size_t max_len) {
    char tx_buffer[BUFFER_SIZE];
    snprintf(tx_buffer, sizeof(tx_buffer), "%s\r\n", cmd);

#if defined(_WIN32) || defined(_WIN64)
    DWORD bytes_written = 0, bytes_read = 0;
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    if (!WriteFile(handle, tx_buffer, (DWORD)strlen(tx_buffer), &bytes_written, NULL)) return -1;

    memset(response_out, 0, max_len);
    DWORD total_bytes = 0;
    char c = 0;

    while (total_bytes < (max_len - 1)) {
        if (ReadFile(handle, &c, 1, &bytes_read, NULL) && bytes_read > 0) {
            response_out[total_bytes++] = c;
            if (c == '\n') break;
        } else break;
    }
    return (total_bytes > 0) ? 0 : -1;
#else
    tcflush(handle, TCIOFLUSH);
    if (write(handle, tx_buffer, strlen(tx_buffer)) < 0) return -1;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(handle, &readfds);

    struct timeval tv = { TIMEOUT_SEC, 0 };
    if (select(handle + 1, &readfds, NULL, NULL, &tv) <= 0) return -1;

    memset(response_out, 0, max_len);
    ssize_t total_read = 0;
    while (total_read < (ssize_t)(max_len - 1)) {
        char c;
        if (read(handle, &c, 1) > 0) {
            response_out[total_read++] = c;
            if (c == '\n') break;
        } else break;
    }
    return (total_read > 0) ? 0 : -1;
#endif
}

// Sensor Bitmap Decoder
void decode_gets_response(const char *raw_response, TAS300_Sensors *sensors) {
    if (!raw_response || !sensors) return;
    memset(sensors, 0, sizeof(TAS300_Sensors));

    const char *ptr = raw_response;
    uint32_t val = 0;
    int found_hex = 0;

    while (*ptr) {
        if (isxdigit((unsigned char)*ptr)) {
            if (sscanf(ptr, "%x", &val) == 1) {
                found_hex = 1;
                break;
            }
        }
        ptr++;
    }

    if (!found_hex) return;

    sensors->raw_hex_value  = (uint16_t)val;
    sensors->info_pad_a     = (val >> 0) & 0x01;
    sensors->info_pad_b     = (val >> 1) & 0x01;
    sensors->info_pad_c     = (val >> 2) & 0x01;
    sensors->info_pad_d     = (val >> 3) & 0x01;
    sensors->placement_left = (val >> 4) & 0x01;
    sensors->placement_right= (val >> 5) & 0x01;
    sensors->presence_front = (val >> 6) & 0x01;
    sensors->presence_rear  = (val >> 7) & 0x01;
    sensors->foup_clamped   = (val >> 8) & 0x01;
    sensors->door_opened    = (val >> 9) & 0x01;
    sensors->z_stage_down   = (val >> 10) & 0x01;
    sensors->emo_active     = (val >> 11) & 0x01;
    sensors->main_air_ok    = (val >> 12) & 0x01;

    printf("\n========================================\n");
    printf("     TAS300 SENSOR DECODE (0x%04X)      \n", sensors->raw_hex_value);
    printf("========================================\n");
    printf("[Info Pads]       A:%d | B:%d | C:%d | D:%d\n",
           sensors->info_pad_a, sensors->info_pad_b, sensors->info_pad_c, sensors->info_pad_d);
    printf("[Placement]      Left: %-3s | Right: %s\n",
           sensors->placement_left ? "ON" : "OFF", sensors->placement_right ? "ON" : "OFF");
    printf("[Presence]       Front: %-2s | Rear: %s\n",
           sensors->presence_front ? "ON" : "OFF", sensors->presence_rear ? "ON" : "OFF");
    printf("[Mechanism]      Clamp: %-3s | Door: %-4s | Z-Down: %s\n",
           sensors->foup_clamped ? "YES" : "NO", sensors->door_opened ? "OPEN" : "CLSD", sensors->z_stage_down ? "YES" : "NO");
    printf("[Interlocks]     Air Supply: %-3s | EMO Active: %s\n",
           sensors->main_air_ok ? "OK" : "LOW", sensors->emo_active ? "YES (ALARM)" : "NO");
    printf("========================================\n\n");
}

// Full Command API Wrapper Suite
int tas300_cmd_org(SerialPortHandle h, char *res)   { return send_tas300_command(h, "ORG", res, BUFFER_SIZE); }
int tas300_cmd_load(SerialPortHandle h, char *res)  { return send_tas300_command(h, "LOAD", res, BUFFER_SIZE); }
int tas300_cmd_unld(SerialPortHandle h, char *res)  { return send_tas300_command(h, "UNLD", res, BUFFER_SIZE); }
int tas300_cmd_mopn(SerialPortHandle h, char *res)  { return send_tas300_command(h, "MOPN", res, BUFFER_SIZE); }
int tas300_cmd_mcls(SerialPortHandle h, char *res)  { return send_tas300_command(h, "MCLS", res, BUFFER_SIZE); }
int tas300_cmd_dock(SerialPortHandle h, char *res)  { return send_tas300_command(h, "DOCK", res, BUFFER_SIZE); }
int tas300_cmd_udck(SerialPortHandle h, char *res)  { return send_tas300_command(h, "UDCK", res, BUFFER_SIZE); }
int tas300_cmd_map(SerialPortHandle h, char *res)   { return send_tas300_command(h, "MAP", res, BUFFER_SIZE); }
int tas300_cmd_getm(SerialPortHandle h, char *res)  { return send_tas300_command(h, "GETM", res, BUFFER_SIZE); }
int tas300_cmd_gets(SerialPortHandle h, char *res)  { return send_tas300_command(h, "GETS", res, BUFFER_SIZE); }
int tas300_cmd_rst(SerialPortHandle h, char *res)   { return send_tas300_command(h, "RST", res, BUFFER_SIZE); }
int tas300_cmd_ver(SerialPortHandle h, char *res)   { return send_tas300_command(h, "VER", res, BUFFER_SIZE); }

// Console Menu
void show_menu(void) {
    printf("\n------- TDK TAS300 MENU (19200 BAUD) -------\n");
    printf(" 1. VER  - Query Firmware Version\n");
    printf(" 2. GETS - Query & Decode Sensor Bitmap\n");
    printf(" 3. ORG  - Execute Homing/Initialization\n");
    printf(" 4. DOCK - Dock FOUP Forward\n");
    printf(" 5. UDCK - Undock FOUP Retract\n");
    printf(" 6. MOPN - Unlatch & Open Door\n");
    printf(" 7. MCLS - Close & Latch Door\n");
    printf(" 8. LOAD - Complete Automated Load Sequence\n");
    printf(" 9. UNLD - Complete Automated Unload Sequence\n");
    printf("10. MAP  - Trigger Wafer Mapping Scan\n");
    printf("11. GETM - Retrieve Wafer Mapping Result\n");
    printf("12. RST  - Clear Active Faults/Alarms\n");
    printf(" 0. EXIT - Close Connection\n");
    printf("-------------------------------------------\n");
    printf("Select option: ");
}

int main(int argc, char *argv[]) {
    const char *port_name = (argc > 1) ? argv[1] : DEFAULT_PORT;
    printf("Connecting to TDK TAS300 Load Port at 19200 baud on %s...\n", port_name);

    SerialPortHandle handle = open_and_config_serial(port_name);
    if (handle == INVALID_PORT_HANDLE) {
        printf("Error: Unable to open serial port %s\n", port_name);
        return EXIT_FAILURE;
    }

    char response[BUFFER_SIZE];
    TAS300_Sensors sensors;
    int choice = -1;

    while (choice != 0) {
        show_menu();
        if (scanf("%d", &choice) != 1) break;

        switch (choice) {
            case 1:
                tas300_cmd_ver(handle, response);
                printf("Response: %s\n", response);
                break;
            case 2:
                if (tas300_cmd_gets(handle, response) == 0) {
                    decode_gets_response(response, &sensors);
                }
                break;
            case 3:
                tas300_cmd_org(handle, response);
                printf("Response: %s\n", response);
                break;
            case 4:
                tas300_cmd_dock(handle, response);
                printf("Response: %s\n", response);
                break;
            case 5:
                tas300_cmd_udck(handle, response);
                printf("Response: %s\n", response);
                break;
            case 6:
                tas300_cmd_mopn(handle, response);
                printf("Response: %s\n", response);
                break;
            case 7:
                tas300_cmd_mcls(handle, response);
                printf("Response: %s\n", response);
                break;
            case 8:
                printf("Executing Full LOAD Sequence...\n");
                tas300_cmd_load(handle, response);
                printf("Response: %s\n", response);
                break;
            case 9:
                printf("Executing Full UNLOAD Sequence...\n");
                tas300_cmd_unld(handle, response);
                printf("Response: %s\n", response);
                break;
            case 10:
                tas300_cmd_map(handle, response);
                printf("Response: %s\n", response);
                break;
            case 11:
                tas300_cmd_getm(handle, response);
                printf("Wafer Map Response: %s\n", response);
                break;
            case 12:
                tas300_cmd_rst(handle, response);
                printf("Response: %s\n", response);
                break;
            case 0:
                printf("Exiting application...\n");
                break;
            default:
                printf("Invalid selection.\n");
                break;
        }
    }

    close_serial_port(handle);
    return EXIT_SUCCESS;
}