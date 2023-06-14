#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <termios.h>

#define SPI_DEVICE "/dev/spidev1.0"  // SPI 장치 경로
#define UART_DEVICE "/dev/ttyS0"     // UART 장치 경로
#define SS_PIN 17                    // SPI SS(Chip Select) 핀 번호
#define RST_PIN 10                    // RST 핀 번호
#define MISO_PIN 21
#define MOSI_PIN 18
#define SCK 26

int spi_fd;
int uart_fd;

// SPI 통신 초기화
int spi_init()
{
    spi_fd = open(SPI_DEVICE, O_RDWR);
    // if (spi_fd < 0) {
    //     perror("Failed to open SPI device");
    //     return -1;
    // }

    // SPI 모드 설정
    uint8_t mode = SPI_MODE_0;
    // if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1) {
    //     perror("Failed to set SPI mode");
    //     return -1;
    // }

    // 최대 전송 속도 설정 (1MHz)
    uint32_t speed = 1000000;
    // if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
    //     perror("Failed to set SPI speed");
    //     return -1;
    // }

    return 0;
}

// SPI 데이터 송수신
int spi_transfer(uint8_t* tx_data, uint8_t* rx_data, int len)
{
    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_data,      // 송신할 데이터가 저장된 버퍼 주소 지정
        .rx_buf = (unsigned long)rx_data,      // 수신된 데이터를 저장할 버퍼 주소 지정 
        .len = len,                            // 송수신할 데이터 길이
        .delay_usecs = 0,
        .speed_hz = 0,
        .bits_per_word = 8,
        .cs_change = 0,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) == -1) {    // 1개의 SPI 메시지 전송할 때, 오류 발생 시 perror 호출
        perror("Failed to perform SPI transfer");
        return -1;
    }

    return 0;
}

// UART 통신 초기화
int uart_init()
{
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);    // O_RDWR : OPEN, READ, WRITE 모두 지원 플래그
    // if (uart_fd < 0) {                                 // O_NOCTTY : 터미널 속성(Baud rate, 비트 수 등)을 설정할 필요없이 간단히 데이터 read/write 가능
    //     perror("Failed to open UART device");
    //     return -1;
    // }

    struct termios options;
    // if (tcgetattr(uart_fd, &options) != 0) {       // tcgetattr : uart_fd로 지정된 UART 디바이스의 현재 설정 속성을 가져옴
    //     perror("Failed to get UART attributes");
    //     return -1;
    // }

    cfsetispeed(&options, B9600);     // 입력 속도 설정
    cfsetospeed(&options, B9600);     // 출력 속도 설정, 송수신 측의 동일한 데이터 속도 맞추기 위함
    cfmakeraw(&options);              // 데이터를 raw한 상태로 초기화하기 위함.

    // if (tcsetattr(uart_fd, TCSANOW, &options) != 0) {    // 터미널 속성 설정, uart_fd 에 대한 터미널 속성을 options 에 저장된 값으로 즉시 설정.
    //     perror("Failed to set UART attributes");
    //     return -1;
    // }

    return 0;
}

// UID 정보 UART로 전송
void send_uid_uart(uint8_t* uid, int len)
{
    write(uart_fd, uid, len);
    write(uart_fd, "\n", 1);
}

void mfrc522_init(int rst_pin)
{
    int rst_pin_fd;
    // RST 핀 초기화
    if (initialize_rst_pin(rst_pin) != 0) {
        exit(1);
    }
    write(rst_pin_fd, "0", 1);    // pin 상태 low
    usleep(500000);
    write(rst_pin_fd, "1", 1);    // pin 상태 high
    usleep(500000);
    close(rst_pin_fd);

    // MFRC522 초기화 코드 작성
    // register, timer, network(coummu) setting etc...

}

void endTagDetection() {
    // 태그 감지 종료 동작 수행
    // 예시로는 다음과 같이 태그 감지 상태를 변경
    isTagDetectionEnabled = false;
}

// MFRC522 초기화 및 UID 정보 읽기
void mfrc522_read_uid()
{
    // MFRC522 초기화 코드 작성
    mfrc522_init(RST_PIN);

// 태그 감지 코드 작성
    while (1) {
        if (isTagPresent()) {
        // 태그가 감지되었을 때의 동작
        break;
        }
        usleep(100000); // 100ms 대기 후 다시 감지
    }

    // UID 정보 읽기 코드 작성
    uint8_t uid[4];
    if (readUID(uid)) {
        // UID 정보 읽기 성공
        // uid 배열에 읽어온 UID 정보가 저장됨
    } else {
        // UID 정보 읽기 실패
    }

    // 읽어온 UID 정보 처리
    
    // UID 정보 전송
    send_uid_uart(uid, uid_len);

    // 태그 종료 코드 작성
void stopTagDetection() {
    endTagDetection();
    // ...
 }
}


int main()
{
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
// if (ss_gpio_fd < 0) {
//     perror("Failed to open SS pin");
//     return 1;
// }

// SS 핀을 출력으로 설정
write(ss_gpio_fd, "out", 3);    // or "in"
                                    // 파일 디스크립터에 out 이라는 문자열을 쓰기 위해, 데이터 길이인 3만큼을 파일에 씀.
// SS 핀에 값 쓰기 (1로 설정)
write(ss_gpio_fd, "1", 1);      // or "0"

// SS 핀 닫기
close(ss_gpio_fd);

// RST 핀 설정
int rst_gpio_fd;
char rst_gpio_path[64];

// RST 핀 설정 및 초기화 함수
int initialize_rst_pin(int rst_pin) {
    // RST 핀 경로 설정
    snprintf(rst_gpio_path, sizeof(rst_gpio_path), "/sys/class/gpio/gpio%d/value", rst_pin);

    // RST 핀 열기
    rst_gpio_fd = open(rst_gpio_path, O_WRONLY);
    // if (rst_gpio_fd < 0) {
    //     perror("Failed to open RST pin");
    //     return -1;
    // }

    // RST 핀을 출력으로 설정
    write(rst_gpio_fd, "out", 3);

    // RST 핀에 값 쓰기 (1로 설정)
    write(rst_gpio_fd, "1", 1);

    return 0;
}

// RST 핀 닫기 함수
close(rst_gpio_fd);

// MOSI 핀 설정
int mosi_gpio_fd;
char mosi_gpio_path[64];

// MOSI 핀 열기
snprintf(mosi_gpio_path, sizeof(mosi_gpio_path), "/sys/class/gpio/gpio%d/value", MOSI_PIN);
mosi_gpio_fd = open(mosi_gpio_path, O_WRONLY);
// if (mosi_gpio_fd < 0) {
//     perror("Failed to open MOSI pin");
//     return 1;
// }

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
// if (miso_gpio_fd < 0) {
//     perror("Failed to open MISO pin");
//     return 1;
// }

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
// if (sck_gpio_fd < 0) {
//     perror("Failed to open SCK pin");
//     return 1;
// }

// SCK 핀을 출력으로 설정
write(sck_gpio_fd, "out", 3);

// SCK 핀 닫기
close(sck_gpio_fd);

    // SPI 통신 초기화
    if (spi_init() != 0) {
        return 1;
    }

    // UART 통신 초기화
    if (uart_init() != 0) {
        return 1;
    }

    // MFRC522 초기화 및 UID 정보 읽기
    while (1) {
        mfrc522_read_uid();
        usleep(500000);
    }

    // 파일 디스크립터 닫기
    close(spi_fd);
    close(uart_fd);

    return 0;
}