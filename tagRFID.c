#include <stdio.h>
#include <string.h>
#include <wiringPi>
#include <wiringSerial.h>
#include <MFRC522.h>

#define SS_PIN 15
#define RST_PIN 14
#define UART_PORT "/dev/ttyS0"

int main () {
// initialize
    wiringPiSetup();
    wiringPiSPISetup(0,1000000);
    MFRC522 mfrc522(SS_PIN, RST_PIN);
    mfrc522.PCD_Init();

    print("RFID-RC522 Reader Initialized");

// UART port open
    int uart=serialOpen(UART_PORT, 9600);
    if (uart == -1) {
        fprintf(stderr, "Failed to open UART port");
        return 1;
    }

// touch tag
    while(1) {
        if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
            printf("RFID Tag Detected \n");

// UID read
    char uid[16];
    memset(uid, 0, sizeof(uid));
    for (int i=0; i<mfrc522.uid.size; i++) {
        sprinf(uid + (i*2), "%02X", mfrc522.uid.uidByte[i]);
    }

// transfer UID info to UART
    serialPuts(uart, uid);
    serialPuts(uart, "\n");

    mfrc522.PICC_HaltA();       // PICC stop
    mfrc522.PCD_StopCrypto1();  // PCD stop
    }
    else if (mfrc522.PICC_ReadCardSerial()) {
        printf("Error : Multiple cards detected");

        serialPuts(uart,"Error : Multiple cards detected");
    }
    delay(500);
}

// UART port close
    serialClose(uart);
    return 0;
}