#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>
#include <rtl-sdr.h>
#include <unistd.h> // For POSIX systems
#include <stdint.h>
#include <math.h>
#include <signal.h>
#include <float.h>
#include <MQTTClient.h>
#include "types.h"
#include "mqtt0client.h"

#define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_TIMEOUT	300
#define MODES_RTL_BUF_SIZE         (16*16384)                 // 256k
#define MODES_LONG_MSG_BYTES     14
#define MODES_SHORT_MSG_BYTES    7
// CRC polynomial for Mode S
#define MODES_POLY 0xfff409U
#define PREAMBLE_LEN 16 // 8us at 2MHz = 16 samples
#define MSG_BITS 112
#define SAMPLES_PER_BIT 2
#define PREAMBLE_THRESH 1 // Adjust as needed
#define MODES_LONG_MSG_BITS 112
#define MODES_SHORT_MSG_BITS 56
#define MODES_FULL_LEN (8+112)


MQTTClient client;

// Convert a hex string (e.g., "8D4840D6202CC371C32CE0576098") to a bits array (MSB first)
void hex_string_to_bits(const char *hex, int *bits, int num_bits) {
    int num_bytes = strlen(hex) / 2;
    for (int i = 0; i < num_bytes; i++) {
        char byte_str[3] = {hex[2 * i], hex[2 * i + 1], 0};
        unsigned char byte = (unsigned char) strtol(byte_str, NULL, 16);
        for (int b = 0; b < 8; b++) {
            int bit_index = i * 8 + b;
            if (bit_index >= num_bits) break;
            bits[bit_index] = (byte & (1 << (7 - b))) ? 1 : 0;
        }
    }
}

// Helper: Pack bits (0/1) into bytes, MSB first
void pack_bits_to_bytes(const int *bits, uint8_t *bytes, int num_bits) {
    memset(bytes, 0, (num_bits + 7) / 8);
    for (int i = 0; i < num_bits; i++) {
        if (bits[i]) {
            bytes[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
}

// Structure to hold RTL device info
typedef struct RTLHandler {
    libusb_device *device; // Pointer to USB device
    libusb_device_handle *handle; // Handle to open device
    unsigned char str[256]; // Product string
} rtl_handler;

rtl_handler *get_rtl_devices(void) {
    libusb_device **devs;
    libusb_context *ctx = NULL;
    ssize_t cnt;
    int r;

    r = libusb_init(&ctx);
    if (r < 0) {
        printf("Init Error %d\n", r);
        return NULL;
    }

    cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        printf("Get Device Error\n");
        libusb_exit(ctx);
        return NULL;
    }


    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(devs[i], &desc);
        // printf("Device %zu: Vendor ID: %04x, Product ID: %04x\n", i, desc.idVendor, desc.idProduct);

        libusb_device_handle *handle = NULL;
        unsigned char str[256];
        if (libusb_open(devs[i], &handle) == 0 && handle) {
            if (desc.iProduct) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iProduct, str, sizeof(str)) > 0) {
                    if (strstr((char *) str, "RTL") != NULL) {
                        rtl_handler *handler = malloc(sizeof(rtl_handler));
                        if (handler) {
                            handler->device = devs[i];
                            handler->handle = handle;
                            memcpy(handler->str, str, sizeof(str));
                        }
                        return handler;
                    }
                }
            }
            libusb_close(handle);
        }
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
}

volatile int do_exit = 0;

void sigint_handler(int signum) {
    do_exit = 1;
}

int number_of_buffers() {
    FILE *f = fopen("/Users/qasim/CLionProjects/output.bin", "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long filesize = ftell(f);
    fclose(f);
    return filesize / MODES_RTL_BUF_SIZE;
}

void record_buff(unsigned char *buf, uint32_t len) {
    FILE *f = fopen("signal_dump.bin", "ab");
    if (f) {
        fwrite(buf, 1, len, f);
        fclose(f);
        printf("Appended %u bytes to signal_dump.bin\n", len);
        sleep(1);
    } else {
        printf("Failed to open file for writing\n");
    }
}

unsigned char *read_buffer(int n) {
    FILE *f = fopen("/Users/qasim/CLionProjects/output.bin", "rb");
    if (!f) {
        printf("Failed to open file for reading\n");
        return NULL;
    }
    fseek(f, n * MODES_RTL_BUF_SIZE, SEEK_SET);
    unsigned char buf[MODES_RTL_BUF_SIZE];
    if (!buf) {
        fclose(f);
        printf("Failed to allocate buffer\n");
        return NULL;
    }
    size_t bytes_read = fread(buf, 1, MODES_RTL_BUF_SIZE, f);
    if (bytes_read < MODES_RTL_BUF_SIZE) {
        printf("Warning: Read only %zu bytes from buffer %d\n", bytes_read, n);
    } else {
        printf("Read %zu bytes from buffer %d\n", bytes_read, n);
    }
    fclose(f);
    return buf;
}

void plot_signal(const float *buf, size_t len) {
    FILE *gp = popen("gnuplot -persistent", "w");
    if (!gp) {
        printf("Failed to open gnuplot\n");
        return;
    }
    fprintf(gp, "set title 'Signal Magnitude'\n");
    fprintf(gp, "plot '-' with lines title 'Magnitude'\n");

    size_t num_samples = len / 2;
    for (size_t i = 0; i < len; i++) {
        // int I = ((int) buf[2 * i]) - 127;
        // int Q = ((int) buf[2 * i + 1]) - 127;
        // double mag = sqrt(I * I + Q * Q);
        fprintf(gp, "%zu %f\n", i, buf[i]);
    }
    fprintf(gp, "e\n");
    fflush(gp);
    getchar();
    fprintf(gp, "exit\n");
    pclose(gp);
}

int is_preamble(const float *mag, int i, float t) {
    // Preamble pattern: high at 0,2,7,9; low at 1,3,4,5,6,8
    if (((mag[i + 0] * 100) > t ? 1 : 0) != 1) return 0;
    if (((mag[i + 2] * 100) > t ? 1 : 0) != 1) return 0;
    if (((mag[i + 7] * 100) > t ? 1 : 0) != 1) return 0;
    // if (((mag[i + 9] * 100) > t ? 1 : 0) != 1) return 0;
    // Quiet spots if (((mag[i + 1] * 100) > 30) != 0) return 0;
    if (((mag[i + 1] * 100) > t) != 0) return 0;
    if (((mag[i + 3] * 100) > t) != 0) return 0;
    if (((mag[i + 4] * 100) > t) != 0) return 0;
    if (((mag[i + 5] * 100) > t) != 0) return 0;
    if (((mag[i + 6] * 100) > t) != 0) return 0;

    // if ((mag[i + 0] + mag[i + 2] + mag[i + 7] * 2) < 3 * (mag[i + 4] + mag[i + 6] + mag[i + 5])) // about 3.5dB SNR
    //     return 0;
    return 1;
}

float to_amplitude(const unsigned char buf[], int bit_index) {
    // Each bit is 2 samples (4 bytes: I0,Q0,I1,Q1)
    float I = (buf[2 * bit_index] - 127.4) / 128;
    float Q = (buf[2 * bit_index + 1] - 127.4) / 128;
    double magsq = I * I + Q * Q;

    float mag0 = sqrt(magsq);

    return mag0;
}

int valid_bit(float magnitude, float th) {
    return (magnitude) * 100 > th ? 1 : 0; // Normalize to 0 or 1
}

// Smooths the input signal using a moving average filter
void moving_average_filter(const float *input, float *output, size_t len, int window) {
    for (size_t i = 0; i < len; i++) {
        float sum = 0.0f;
        int count = 0;
        for (int j = -(window / 2); j <= window / 2; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < len) {
                sum += input[idx];
                count++;
            }
        }
        output[i] = sum / count;
    }
}


double *convertToMagnitude(unsigned char *buf, unsigned int len) {
    /*
     *Yes, one I and Q sample together represent a single point in time—specifically, both are captured at the same instant.
     *At a 2 MHz sample rate (used for ADS-B), each I/Q pair represents a sample every 0.5 microseconds.
     *So, each I and Q value in the pair corresponds to the same 0.5 microsecond interva
     */
    double *mag = malloc((len / 2) * sizeof(double));
    for (int i = 0; i < len / 2; i++) {
        float I = (buf[2 * i] - 127.4) / 128;
        float Q = (buf[i * 2 + 1] - 127.4) / 128;
        double magsq = I * I + Q * Q;
        double sqt = sqrt(magsq);
        mag[i] = sqt;
    }
    return mag;
}

double maximum(double *mag, unsigned int len) {
    double max = 0.0;
    for (unsigned int i = 0; i < len / 2; i++) {
        max = fmax(max, mag[i]);
    }
    return max;
}

void normalize(double *mag, unsigned int len) {
    double max = maximum(mag, len);
    for (unsigned int i = 0; i < len / 2; i++) {
        mag[i] /= max;
    }
}

double calculateThreshold(double *mag, unsigned int len) {
    double max = maximum(mag, len);
    return max * .5f;
}

int checkPuls(double mag, double threshold) {
    return (mag) >= threshold ? 1 : 0; // Normalize to 0 or 1
}

int toBit(double *mag, int i, double threshold) {
    i = i * 2;
    if ((mag[i] > mag[i + 1]) && (mag[i] > threshold)) {
        return 1; // High pulse
    }
    if ((mag[i] < mag[i + 1]) && (mag[i] < threshold)) {
        return 0; // Low pulse
    }
    if (mag[i] > mag[i + 1]) {
        return 1; // High pulse
    }
    if (mag[i] < mag[i + 1]) {
        return 0; // Low pulse
    }
    return -1;
}

double otsu_threshold(const double *mag, size_t len) {
    int hist_bins = 256;
    int hist[256] = {0};
    double min = DBL_MAX, max = -DBL_MAX;

    // Find min and max
    for (size_t i = 0; i < len; i++) {
        if (mag[i] < min) min = mag[i];
        if (mag[i] > max) max = mag[i];
    }
    if (max == min) return min; // All values same

    // Build histogram
    for (size_t i = 0; i < len; i++) {
        int bin = (int) (((mag[i] - min) / (max - min)) * (hist_bins - 1));
        if (bin < 0) bin = 0;
        if (bin >= hist_bins) bin = hist_bins - 1;
        hist[bin]++;
    }

    // Otsu's algorithm
    int total = (int) len;
    double sum = 0;
    for (int t = 0; t < hist_bins; t++)
        sum += t * hist[t];

    double sumB = 0, wB = 0, wF = 0, varMax = 0;
    int threshold = 0;
    for (int t = 0; t < hist_bins; t++) {
        wB += hist[t];
        if (wB == 0) continue;
        wF = total - wB;
        if (wF == 0) break;
        sumB += t * hist[t];
        double mB = sumB / wB;
        double mF = (sum - sumB) / wF;
        double varBetween = wB * wF * (mB - mF) * (mB - mF);
        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }
    // Convert threshold bin to value
    return min + (threshold / (double) (hist_bins - 1)) * (max - min);
}

int isItPreamble(double *mag, double threshold) {
    // High at 0, 2, 7, 9
    if (checkPuls(mag[0], threshold) != 1) return 0;
    if (checkPuls(mag[2], threshold) != 1) return 0;
    if (checkPuls(mag[7], threshold) != 1) return 0;
    if (checkPuls(mag[9], threshold) != 1) return 0;
    // Low at 1, 3, 4, 5, 6, 8
    if (checkPuls(mag[1], threshold) != 0) return 0;
    if (checkPuls(mag[3], threshold) != 0) return 0;
    if (checkPuls(mag[4], threshold) != 0) return 0;
    if (checkPuls(mag[5], threshold) != 0) return 0;
    if (checkPuls(mag[6], threshold) != 0) return 0;
    if (checkPuls(mag[8], threshold) != 0) return 0;

    return 1;
}

FILE *gp = NULL;

void open_gnuplot() {
    if (!gp) {
        gp = popen("gnuplot -persistent", "w");
        fprintf(gp, "set title 'Signal Magnitude'\n");
        fprintf(gp, "plot '-' with lines title 'Magnitude'\n");
        fflush(gp);
    }
}

void plot_signal_runtime(const double *buf, size_t len) {
    if (!gp) open_gnuplot();
    fprintf(gp, "set title 'Signal Magnitude'\n");
    fprintf(gp, "plot '-' with lines title 'Magnitude'\n");
    for (size_t i = 0; i < len; i++) {
        fprintf(gp, "%zu %f\n", i, buf[i]);
    }
    fprintf(gp, "e\n");
    fflush(gp);
    // No getchar() here; call as needed in your loop
}


void close_gnuplot() {
    if (gp) {
        fprintf(gp, "exit\n");
        pclose(gp);
        gp = NULL;
    }
}

// Mode S generator polynomial (24 bits)
#define MODES_POLY 0xFFF409U

// Compute 24-bit CRC for a 112-bit (14-byte) Mode S message
uint32_t modes_crc(const uint8_t *msg) {
    // Copy message bits to a 112-bit buffer (as 1 bit per byte)
    uint8_t bits[112];
    for (int i = 0; i < 112; i++) {
        bits[i] = msg[i];
    }

    // Append 24 zero bits (total 136 bits)
    uint8_t data[112] = {0};
    memcpy(data, bits, 112);

    // Generator as 25 bits (MSB first)
    uint32_t generator = 0x1FFF409; // 25 bits: 1111111111111010000001001

    // CRC division
    for (int i = 0; i < 88; i++) {
        if (data[i]) {
            for (int j = 0; j < 25; j++) {
                data[i + j] ^= (generator >> (24 - j)) & 1;
            }
        }
    }

    // Extract remainder (last 24 bits)
    uint32_t crc = 0;
    for (int i = 0; i < 24; i++) {
        crc = (crc << 1) | data[88 + i];
    }
    return crc;
}

uint32_t decode_block(int start, int end, const unsigned char *msg) {
    uint32_t value = 0;
    for (int i = start; i < end; i++) {
        value <<= 1;
        if (msg[i]) {
            value |= 1;
        }
    }
    return value;
}

char *decodeFlightNumber(unsigned char *bits) {
    uint32_t C1 = decode_block(40, 46, bits);
    uint32_t C2 = decode_block(46, 52, bits);
    uint32_t C3 = decode_block(52, 58, bits);
    uint32_t C4 = decode_block(58, 64, bits);
    uint32_t C5 = decode_block(64, 70, bits);
    uint32_t C6 = decode_block(70, 76, bits);
    uint32_t C7 = decode_block(70, 76, bits);
    uint32_t C8 = decode_block(70, 76, bits);

    char map[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";
    unsigned char *flight = malloc(9 * sizeof(char));
    printf("%c%c%c-%c%c%c%c%c\n", map[C1], map[C2], map[C3], map[C4], map[C5], map[C6], map[C7], map[C8]);
    sprintf(flight, "%c%c%c%c%c%c%c%c\0",
            map[C1], map[C2], map[C3], map[C4], map[C5], map[C6], map[C7], map[C8]);
    return flight;
}


char *bits_to_hex_string(const unsigned char *bits, int num_bits) {
    int num_bytes = (num_bits + 7) / 8;
    unsigned char *bytes = malloc(num_bytes);
    memset(bytes, 0, num_bytes);

    for (int i = 0; i < num_bits; i++) {
        if (bits[i]) {
            bytes[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    char *hex_str = malloc(num_bytes * 2 + 1);
    for (int i = 0; i < num_bytes; i++) {
        sprintf(hex_str + i * 2, "%02X", bytes[i]);
    }
    hex_str[num_bytes * 2] = '\0';

    free(bytes);
    return hex_str;
}

void async_callback(unsigned char *buf, unsigned int len, void *ctx) {
    // detectModeS(buf, len);
    double *magnitude = convertToMagnitude(buf, len);
    // normalize(magnitude, len / 2);
    int magnitudelength = (len / 2) - PREAMBLE_LEN - MSG_BITS;
    double threshold = otsu_threshold(magnitude, len / 2);
    for (int i = 0; i < magnitudelength; i++) {
    skip_preamble:
        if (isItPreamble(magnitude + i, threshold)) {
            threshold = otsu_threshold(magnitude + 1, PREAMBLE_LEN + 112);

            double *message = magnitude + i + PREAMBLE_LEN;

            unsigned char bits[112] = {0}; // 112 bits for Mode S message
            for (int b = 0; b < MSG_BITS; b++) {
                bits[b] = toBit(message, b, threshold);
                if (bits[b] == -1) {
                    goto skip_preamble;
                }
            }


            unsigned int df = decode_block(0, 5, bits);
            unsigned int parity = decode_block(88, 112, bits);
            unsigned int crc = modes_crc(bits);

            if (crc) {
                continue;
            }


            RawMessage *rawMsg = (RawMessage *) malloc(sizeof(RawMessage));
            rawMsg->msg = malloc(28 * sizeof(char));
            strcpy(rawMsg->msg, bits_to_hex_string(bits,112));
            printf("Raw Message: %s\n", rawMsg->msg);

            char *jsonStr = toRawMessageStr(*rawMsg);
            mqttPublish(client, jsonStr);

            free(jsonStr);
            free(rawMsg);

            if (df == 17) {
                printf("\nDF (5 bits): %d\n", df);
                unsigned int ca = decode_block(5, 8, bits);
                unsigned int type_code = decode_block(32, 37, bits);
                uint32_t icao = decode_block(8, 32, bits);

                printf("ICAO (24 bits): %06X\n", icao & 0xFFFFFF);
                printf("PARITY (24 bits): %06X\n", parity & 0xFFFFFF);
                printf("Type Code (5 bits): %d\n", type_code);
                printf("CA (3 bits): %d\n", ca);


                switch (type_code) {
                    case 4: {
                        uint32_t EC = decode_block(37, 40, bits);
                        printf("%s\n", decodeFlightNumber(bits));
                        break;
                    }
                    case 19: {
                        uint32_t subType = decode_block(37, 40, bits);

                        if (subType == 1) {
                            uint32_t IntentChangeFlag = decode_block(40, 41, bits);
                            uint32_t eastWestVelocitySign = decode_block(45, 46, bits);
                            uint32_t eastWestVelocity = decode_block(46, 56, bits);
                            uint32_t northWestVelocitySign = decode_block(56, 57, bits);
                            uint32_t northWestVelocity = decode_block(57, 67, bits);

                            int ew_vel = eastWestVelocitySign ? -eastWestVelocity : eastWestVelocity;
                            int nw_vel = northWestVelocitySign ? -northWestVelocity : northWestVelocity;
                            double ground_speed = sqrt(ew_vel * ew_vel + nw_vel * nw_vel);
                            printf("Ground Speed: %.2f knots\n", ground_speed);
                        }
                        break;
                    }
                    case 11: {
                        uint32_t time = decode_block(52, 53, bits);
                        uint32_t CPR = decode_block(53, 54, bits);
                        uint32_t surveillanceType = decode_block(37, 39, bits);
                        uint32_t latCpr = decode_block(54, 71, bits);
                        uint32_t longCpr = decode_block(71, 88, bits);

                        break;
                    }
                }
            }
        }
    }
    // close_gnuplot();
}


int main() {
    rtlsdr_dev_t *dev = NULL;
    int device_index = 0; // Use 0 for the first RTL device

    if (rtlsdr_open(&dev, device_index) < 0) {
        printf("No RTL-SDR devices found\n");
        return 1;
    }

    printf("Found RTL-SDR device\n");

    // Set center frequency to 1090 MHz (example)
    if (rtlsdr_set_center_freq(dev, 1090000000) == 0) {
        printf("Tuned to 1090 MHz\n");
        rtlsdr_set_sample_rate(dev, 2000000); // 2 MHz sample rate
        // Set gain to 49.6 dB
        // rtlsdr_set_tuner_gain(dev, 496)
        if (rtlsdr_set_tuner_gain(dev, 496) == 0) {
            printf("Gain set to 49.6 dB\n");
        } else {
            printf("Failed to set gain\n");
        }
    } else {
        printf("Failed to set frequency\n");
    }

    // Reset buffer before reading samples
    rtlsdr_reset_buffer(dev);

    signal(SIGINT, sigint_handler);
    printf("Starting async read... Press Ctrl+C to stop.\n");
    int async_result = rtlsdr_read_async(dev, async_callback, NULL, 4, MODES_RTL_BUF_SIZE);
    if (async_result < 0) {
        printf("rtlsdr_read_async failed: %d\n", async_result);
    }

    rtlsdr_close(dev);
    MQTTClient_disconnect(client,TIMEOUT);
    MQTTClient_destroy(client);

    return 0;
}


