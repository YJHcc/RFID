#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <termios.h>  // UART 관련 헤더 파일 추가

#define SPI_DEVICE "/dev/spidev1.0"  // SPI 장치 경로(X: 버스 번호, Y: 장치 번호)
#define UART_DEVICE "/dev/ttyS0"     // UART 장치 경로
//#define RST_PIN 9                   // RST 핀 번호
#define SS_PIN 28                   // SPI SS(Chip Select) GPIO 핀 번호
#define MOSI_PIN 40
#define MISO_PIN 39
#define SCK_PIN 118

// the device tree source needs to be Modified first as follow.
// BONE_PIN(P9_17, spi, P9_17A(PIN_OUTPUT, 4), P9_17B(PIN_INPUT, 7));  // spi6_cs0
// BONE_PIN(P9_18, spi, P9_18A(PIN_OUTPUT, 4), P9_18B(PIN_INPUT, 7));  // spi6_d1, MOSI
// BONE_PIN(P9_21, spi, P9_21A(PIN_INPUT, 4), P9_21B(PIN_INPUT, 7));  // spi6_d0, MISO
// BONE_PIN(P9_26, spi, P9_26A(PIN_OUTPUT, 0), P9_26B(PIN_INPUT, 7));  // spi1_clk

#define MFRC522_PICC_TYPE_MIFARE_MINI 0x09
#define MFRC522_PICC_TYPE_MIFARE_1K 0x04
#define MFRC522_PICC_TYPE_MIFARE_4K 0x02

int spi_fd;
int gpio_fd;
int uart_fd;  // UART 파일 디스크립터 추가

/*  softreset 방법으로 변경
int InitializeRstPin(int RST_PIN) {
    // RST 핀 설정 및 초기화 함수
    int rst_gpio_fd;
    char rst_gpio_path[64];

    // RST 핀 경로 설정
    snprintf(rst_gpio_path, sizeof(rst_gpio_path), "/sys/class/gpio/gpio%d/value", RST_PIN);

    // RST 핀 열기
    rst_gpio_fd = open(rst_gpio_path, O_WRONLY);

    // RST 핀을 출력으로 설정
    write(rst_gpio_fd, "out", 3);

    // RST 핀에 값 쓰기 (1로 설정)
    write(rst_gpio_fd, "1", 1);

    // RST 핀 닫기 함수
    close(rst_gpio_fd);
    return rst_gpio_fd;
}
*/

// SPI 통신 초기화
int SpiInit() {
    spi_fd = open(SPI_DEVICE, O_RDWR);

    // SPI 모드 설정
    uint8_t mode = SPI_MODE_0;
    // ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

    // 최대 전송 속도 설정 (1MHz)
    uint32_t speed = 1000000;
    // ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return 0;
}

// SPI 데이터 송수신
int SpiTransfer(uint8_t* tx_data, uint8_t* rx_data, int len) {
    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_data,  // 송신할 데이터가 저장된 버퍼 주소 지정
        .rx_buf = (unsigned long)rx_data,  // 수신된 데이터를 저장할 버퍼 주소 지정
        .len = len,                        // 송수신할 데이터의 길이 지정
        .delay_usecs = 0,
        .speed_hz = 1000000,
        .bits_per_word = 8,
    };
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
        printf("SPI transfer error\n");
        return -1;
    }
    return 0;
}

// MFRC522 초기화 (SoftReset 이용, SPI 통신을 통해 내부 레지스터를 초기화)
/*
MFRC522 모듈을 초기화 하는 이유
 - 예기치 않은 동작, 이전에 처리되었거나 남아있는 카드 상태 정보를 
   초기화하고 새로운 카드 감지.
*/
void MFRC522Init() { 
    uint8_t tx_data[] = {0x0F}; // MFRC522에 softreset 명령어를 전송하기 위한 데이터 값
    uint8_t rx_data[sizeof(tx_data)];

    if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {
        printf("Failed to send SPI message\n");
        return;
    }
    usleep(5000); // 잠시 대기
}

// 카드가 존재하는지 확인
int PiccIsNewCardPresent() {
    uint8_t tx_data[] = {0x52};   // MFRC522에 해당 함수를 전송하기 위한 데이터 패턴 값, 0x52 : find all the cards antenna area
    uint8_t rx_data[sizeof(tx_data)];
    memset(rx_data, 0, sizeof(rx_data));
    if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {   // SPI 통신 실패 시 카드 존재 여부 확인 불가를 판별하기 위함.
        printf("Failed to transfer data\n");
        return -1;
    }
    return rx_data[1];
}

// 카드 UID 정보 읽기
int PiccReadCardSerial(uint8_t* uid) {
    uint8_t tx_data[] = {0x93};   // MFRC522에 해당 함수를 전송하기 위한 데이터 패턴 값, 0x93 : election card
    uint8_t rx_data[sizeof(tx_data)];
    memset(rx_data, 0, sizeof(rx_data));
    if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {
        printf("Failed to transfer data\n");
        return -1;
    }
    memcpy(uid, &rx_data[1], 4);
    return 0;
}

// 카드 타입 가져오기
uint8_t PiccGetType(uint8_t sak) { //  select Acknowledge, 카드 유형 정보를 나타내는 7비트 값 반환
    return sak & 0x7F;
}

// 카드 타입명 가져오기
const char* PiccGetTypeName(uint8_t piccType) {
    switch (piccType) {
        case MFRC522_PICC_TYPE_MIFARE_MINI:
            return "MIFARE Mini";
        case MFRC522_PICC_TYPE_MIFARE_1K:
            return "MIFARE 1K";
        case MFRC522_PICC_TYPE_MIFARE_4K:
            return "MIFARE 4K";
        default:
            return "Unknown";
    }
}

// 16진수로 변환하여 출력
void printHex(uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        printf("%02X", data[i]);
    }
}

// UART 초기화
int UartInit() {
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
    if (uart_fd < 0) {
        printf("Failed to open UART\n");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);   // import current setting

    // 전송 속도 설정 (9600 bps)
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // 데이터 비트, 패리티 비트, 정지 비트 설정
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 제어 옵션 설정
    // options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(uart_fd, TCSANOW, &options);   // apply new settings to uart_fd

    return 0;
}

// UART로 데이터 전송
void WriteUidUart(const char* data) {
    write(uart_fd, data, strlen(data));
}

int main() {
    // SS 핀 설정
    int ss_gpio_fd;             // fd : 파일 디스크립터로, 특정 파일에 접근할 때 사용하는 추상적인 값
    char ss_gpio_path[64];

// SS 핀 열기
/*
    * snprintf : 버퍼 오버플로우 방지 위해, 버퍼의 크기를 명시적으로 지정하는 문자열 생성.
    * ss_gpio_path라는 문자열에 ss_gpio_path 크기를 가지며, gpio 관련 정보의 경로는 /sys/class/gpio/ 이며,
    * gpio%d/ %d의 숫자로 gpio핀 번호를 정하고,
    * gpio핀의 값을 읽거나 쓰는 파일인 value 파일 내의 gpio핀 상태(0 또는 1)을 설정하여 문자열을 생성
*/

    snprintf(ss_gpio_path, sizeof(ss_gpio_path), "/sys/class/gpio/gpio%d/value", SS_PIN);
    ss_gpio_fd = open(ss_gpio_path, O_WRONLY);  // O_WRONLY : 파일을 쓰기 전용으로 열기 플래그

// SS 핀을 출력으로 설정
    write(ss_gpio_fd, "out", 3);    // or "in"
                                    // 파일 디스크립터에 out 이라는 문자열을 쓰기 위해, 데이터 길이인 3만큼을 파일에 씀.

// RST 핀 설정 및 초기화 함수 call
//    InitializeRstPin(RST_PIN);

// MOSI 핀 설정
    int mosi_gpio_fd;
    char mosi_gpio_path[64];

// MOSI 핀 열기
    snprintf(mosi_gpio_path, sizeof(mosi_gpio_path), "/sys/class/gpio/gpio%d/value", MOSI_PIN);
    mosi_gpio_fd = open(mosi_gpio_path, O_WRONLY);

// MOSI 핀을 출력으로 설정
    write(mosi_gpio_fd, "out", 3);

// MOSI 핀 닫기
    close(mosi_gpio_fd);

// MISO 핀 설정
    int miso_gpio_fd;
    char miso_gpio_path[64];

// MISO 핀 열기
    snprintf(miso_gpio_path, sizeof(miso_gpio_path), "/sys/class/gpio/gpio%d/value", MISO_PIN);
    miso_gpio_fd = open(miso_gpio_path, O_WRONLY);

// MISO 핀을 입력으로 설정
    write(miso_gpio_fd, "in", 2);

// MISO 핀 닫기
    close(miso_gpio_fd);

// SCK 핀 설정
    int sck_gpio_fd;
    char sck_gpio_path[64];

// SCK 핀 열기
    snprintf(sck_gpio_path, sizeof(sck_gpio_path), "/sys/class/gpio/gpio%d/value", SCK_PIN);
    sck_gpio_fd = open(sck_gpio_path, O_WRONLY);

// SCK 핀을 출력으로 설정
    write(sck_gpio_fd, "out", 3);

// SCK 핀 닫기
    close(sck_gpio_fd);

    // SPI 통신 초기화
    if (SpiInit() != 0) {
        return -1;
    }

    // MFRC522 초기화
    MFRC522Init();

    // UART 통신 초기화
    if (UartInit() != 0) {
        return -1;
    }

    uint8_t nuidPICC[4] = {0};  // 이전 카드 UID 저장

    while (1) {
        // 카드가 인식되었다면 다음으로 넘어가고 아니면 더이상 실행하지 않음
        if (!PiccIsNewCardPresent())
            continue;

        // ID가 읽혀졌다면 다음으로 넘어가고 아니면 더이상 실행하지 않음
        if (!PiccReadCardSerial(nuidPICC))
            continue;

        // SS 핀에 값 쓰기 (0으로 설정)
        write(ss_gpio_fd, "0", 1);  // low

        printf("PICC type: ");

        // 카드의 타입을 읽어옴
        uint8_t piccType = PiccGetType(nuidPICC[0]);

        // 모니터에 출력
        printf("%s\n", PiccGetTypeName(piccType));

        // MIFARE 방식인지 확인하고 아니면 리턴
        if (piccType != MFRC522_PICC_TYPE_MIFARE_MINI &&
            piccType != MFRC522_PICC_TYPE_MIFARE_1K &&
            piccType != MFRC522_PICC_TYPE_MIFARE_4K) {
            printf("Your tag is not of type MIFARE Classic.\n");
            continue;
        }

        // 만약 바로 전에 인식한 RF 카드와 다를 경우. 즉, 카드의 중복 감지를 방지하기 위함.
        if (nuidPICC[0] != nuidPICC[0] ||
            nuidPICC[1] != nuidPICC[1] ||
            nuidPICC[2] != nuidPICC[2] ||
            nuidPICC[3] != nuidPICC[3]) {
            printf("A new card has been detected.\n");

            // ID를 저장해둠
            memcpy(nuidPICC, nuidPICC, sizeof(nuidPICC));

            // 모니터 출력
            printf("The NUID tag is:\n");
            printf("In hex: ");
            // 16진수로 변환하여 출력
            printHex(nuidPICC, sizeof(nuidPICC));
            printf("\n");

            // UART를 통해 카드 정보를 PC로 전송
            char uartData[9];
            snprintf(uartData, sizeof(uartData), "%02X%02X%02X%02X\n", nuidPICC[0], nuidPICC[1], nuidPICC[2], nuidPICC[3]);
            WriteUidUart(uartData);
        }

        // SS 핀에 값 쓰기 (1로 설정)
        write(ss_gpio_fd, "1", 1);  // high

        // PICC 종료
        usleep(100000);
    }

    close(spi_fd);
    close(uart_fd);

    // SS 핀 닫기
    close(ss_gpio_fd);
    return 0;
}
