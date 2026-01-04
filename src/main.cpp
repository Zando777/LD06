#include <Arduino.h>

// LD06 LIDAR configuration
// GPIO 16 = RX (connect to LIDAR TX)
// GPIO 4 = TX (not used by LIDAR, but defined for Serial2)
// GPIO 5 = PWM output to control LIDAR motor
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 4
#define LIDAR_PWM_PIN 5
#define LIDAR_BAUD 230400

// PWM settings for motor control
#define PWM_FREQ 10000
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_DUTY 255  // Full speed (0-255)

// LD06 packet constants
#define HEADER 0x54
#define VERLEN 0x2C
#define PACKET_SIZE 47
#define POINTS_PER_PACKET 12

// Packet buffer
uint8_t packet[PACKET_SIZE];
int packet_idx = 0;
bool in_packet = false;

// CRC lookup table for LD06
static const uint8_t crc_table[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

uint8_t calc_crc(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc_table[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

void process_packet() {
    // Verify CRC
    uint8_t crc = calc_crc(packet, PACKET_SIZE - 1);
    if (crc != packet[PACKET_SIZE - 1]) {
        return;
    }

    // Extract angles (0.01 degree resolution)
    uint16_t start_angle = packet[4] | (packet[5] << 8);
    uint16_t end_angle = packet[42] | (packet[43] << 8);

    // Calculate angle step
    float angle_step;
    if (end_angle >= start_angle) {
        angle_step = (float)(end_angle - start_angle) / (POINTS_PER_PACKET - 1);
    } else {
        angle_step = (float)(36000 - start_angle + end_angle) / (POINTS_PER_PACKET - 1);
    }

    // Process each point
    for (int i = 0; i < POINTS_PER_PACKET; i++) {
        int offset = 6 + i * 3;
        uint16_t distance = packet[offset] | (packet[offset + 1] << 8);
        uint8_t confidence = packet[offset + 2];

        // Calculate angle for this point
        float angle = (start_angle + angle_step * i) / 100.0;
        if (angle >= 360.0) {
            angle -= 360.0;
        }

        // Only output valid points (distance > 0, reasonable confidence)
        if (distance > 0 && confidence > 100) {
            // Output format: angle,distance,confidence
            Serial.printf("%.2f,%u,%u\n", angle, distance, confidence);
        }
    }
}

void setup() {
    // USB serial for output to PC
    Serial.begin(921600);
    while (!Serial) {
        delay(10);
    }

    // LIDAR serial
    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

    // Setup PWM for LIDAR motor
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LIDAR_PWM_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, PWM_DUTY);

    Serial.println("LD06 LIDAR Ready");
    Serial.println("PWM motor control started on GPIO 5");
    delay(500);  // Give motor time to spin up
}

void loop() {
    static unsigned long last_debug = 0;
    static int bytes_received = 0;

    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        bytes_received++;

        if (!in_packet) {
            // Look for header
            if (byte == HEADER) {
                packet[0] = byte;
                packet_idx = 1;
                in_packet = true;
            }
        } else {
            packet[packet_idx++] = byte;

            // Check for VerLen byte
            if (packet_idx == 2 && byte != VERLEN) {
                // Invalid packet, reset
                in_packet = false;
                packet_idx = 0;
                continue;
            }

            // Full packet received
            if (packet_idx >= PACKET_SIZE) {
                process_packet();
                in_packet = false;
                packet_idx = 0;
            }
        }
    }

    // Debug: print bytes received every second
    if (millis() - last_debug > 1000) {
        Serial.printf("DEBUG: %d bytes/sec\n", bytes_received);
        bytes_received = 0;
        last_debug = millis();
    }
}
