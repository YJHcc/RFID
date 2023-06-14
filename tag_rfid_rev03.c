// revised tag detection of tagRFID_2.c
// use a changed plan section of RFID_run plan.txt

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <termios.h>

#define SPI_DEVICE "/dev/spidev1.0"  // SPI 장치 경로
#define UART_DEVICE "/dev/ttyS0"     // UART 장치 경로
#define SS_PIN 17                    // SPI SS(Chip Select) 핀 번호
#define RST_PIN 10                   // RST 핀 번호
#define MISO_PIN 21
#define MOSI_PIN 18
#define SCK_PIN 26

int spi_fd;
int uart_fd;
bool isTagDetectionEnabled = true;
uint8_t infoUID[4];
uint8_t uid[4];
// SS 핀 설정
int ss_gpio_fd;             // fd : 파일 디스크립터로, 특정 파일에 접근할 때 사용하는 추상적인 값
char ss_gpio_path[64];


// BBAI-64 and MFRC522 SPI 통신 초기화
int spi_init() {
    spi_fd = open(SPI_DEVICE, O_RDWR);

    // SPI 모드 설정
    uint8_t mode = SPI_MODE_0;

    // 최대 전송 속도 설정 (1MHz)
    uint32_t speed = 1000000;

    return 0;
}

// SPI 데이터 송수신
int spi_transfer(uint8_t* tx_data, uint8_t* rx_data, int len) {
    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_data,      // 송신할 데이터가 저장된 버퍼 주소 지정
        .rx_buf = (unsigned long)rx_data,      // 수신된 데이터를 저장할 버퍼 주소 지정 
        .len = len,                            // 송수신할 데이터 길이
        .delay_usecs = 0,
        .speed_hz = 0,
        .bits_per_word = 8,
        .cs_change = 0,
    };
    // 1개의 SPI 메시지 전송할 때, 오류 발생 시 perror 호출
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) == -1) {
        perror("Failed to perform SPI transfer");
        return -1;
    }

    return 0;
}

// MFRC522 초기화
void mfrc522_init(int rst_pin) {
// RST 핀 초기화
    char rst_pin_path[64];
    snprintf(rst_pin_path, sizeof(rst_pin_path), "/sys/class/gpio/gpio%d/value", rst_pin);
    int rst_pin_fd = open(rst_pin_path, O_WRONLY);
    if (rst_pin_fd < 0) {
        perror("Failed to open RST pin");
        exit(1);
    }
    write(rst_pin_fd, "0", 1);  // pin 상태 low
    usleep(500000);
    write(rst_pin_fd, "1", 1);  // pin 상태 high
    usleep(500000);
    close(rst_pin_fd);
}

// print and conversion red info to hex
void print_hex_message(const uint8_t* data, int length) {
    char hex_buffer[length*2+1];
    for (int i=0; i<length; i++) {
        snprintf(hex_buffer+(i*2), 3, "%02X", data[i]);
    }
    write(spi_fd, hex_buffer, strlen(hex_buffer));
    write(spi_fd, "\n", 1);
}

// MFRC522 모듈과 통신되도록 작성
uint8_t command = {0x0A};  // 태그가 감지되면 MFRC522 모듈에게 "READ_UID" 명령을 전송하여 UID 정보를 요청
// uint8_t uid[4];
void readUID(uint8_t* uid) {
    // 태그 읽기 명령을 전송
    if (command >= 0x00 && command<=0xFF) {
    	uint8_t response[4];  // UID를 저장할 배열

    	// SS 핀을 LOW로 설정하여 MFRC522 모듈과 통신 시작
    	write(ss_gpio_fd, "0", 1);

    	// 명령을 MFRC522 모듈로 전송
    	spi_transfer(command, NULL, sizeof(command));

    	// 응답을 MFRC522 모듈로부터 수신
    	spi_transfer(NULL, response, sizeof(response));

    	// SS 핀을 HIGH로 설정하여 통신 종료
    	write(ss_gpio_fd, "1", 1);

    	// 수신된 UID 정보를 uid 배열에 복사
   	    for (int i = 0; i < 4; i++) {
   	        uid[i] = response[i];
  	    }

        endTagDetection();
   	    // if (readUID(uid)==1) {
	    //     return true;    	// UID 정보 읽기 성공 시 true 반환
  	    // } else {
  	    //     return false;    	// UID 정보 읽기 실패 시 false 반환
  	    // }
    }
}

// 태그 종료 코드 작성
void endTagDetection() {
    bool isTagDetectionEnabled = false;   // 태그 감지 종료 동작 수행
}

// UID 정보 읽기(태그 감지, UID 정보 읽기, 읽어온 UID 정보 처리, UID 정보 전송, 태그 종료)
void mfrc522_read_uid() {
    mfrc522_init(RST_PIN);

// 태그 감지
    while (isTagDetectionEnabled) {
// UID 정보 읽기
        readUID(uid);
        usleep(500000); // 0.5sec 대기 후 다시 감지
    }
}

// UID 정보 UART로 전송
void send_uid_uart(uint8_t* uid, int len) {
    write(uart_fd, uid, len);
    write(uart_fd, "\n", 1);
}

// UID 정보 전송
//send_uid_uart(uid, uid_len);

// UART 통신 초기화
int uart_init() {
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY); // O_RDWR : OPEN, READ, WRITE 모두 지원 플래그
                                                    // O_NOCTTY : 터미널 속성(Baud rate, 비트 수 등)을 설정할 필요없이 간단히 데이터 read/write 가능

    struct termios options;
    // tcgetattr : uart_fd로 지정된 UART 디바이스의 현재 설정 속성을 가져옴

    cfsetispeed(&options, B9600);   // 입력 속도 설정
    cfsetospeed(&options, B9600);   // 출력 속도 설정, 송수신 측의 동일한 데이터 속도 맞추기 위함
    cfmakeraw(&options);            // 데이터를 raw한 상태로 초기화하기 위함
    return 0;
}

int initialize_rst_pin(int rst_pin) {
    // RST 핀 설정 및 초기화 함수
    int rst_gpio_fd;
    char rst_gpio_path[64];

    // RST 핀 경로 설정
    snprintf(rst_gpio_path, sizeof(rst_gpio_path), "/sys/class/gpio/gpio%d/value", rst_pin);

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

int main() {

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
// SS 핀에 값 쓰기 (1로 설정)
    write(ss_gpio_fd, "1", 1);      // or "0"

// SS 핀 닫기
    close(ss_gpio_fd);

// RST 핀 설정 및 초기화 함수 call
    initialize_rst_pin(RST_PIN);

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

// UID 정보 UART로 전송
    send_uid_uart(uart_fd, length(uart_fd));

// 파일 디스크립터 닫기
    close(spi_fd);
    close(uart_fd);

    return 0;
}