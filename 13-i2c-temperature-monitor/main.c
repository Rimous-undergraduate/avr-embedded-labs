#define F_CPU          16000000UL             // CPU 클록 값 = 16 MHz
#define F_SCK          40000UL                  // SCK 클록 값 = 40 KHz #include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>                       // interrupt 관련
#include <avr/io.h>
#define LM75A_ADDR 0x90 // 0b1001_0000, A2, A1, A0 모두 GND (not short), 7비트를 1비트 left shift
#define LM75A_CONFIG_REG    1
#define LM75A_TEMP_REG         0
#define STOP  0  // 체온계 모드
#define GO  1  // 온도계 모드
#define ON 1
#define OFF 0

int sw=0;
unsigned char digit[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67};
volatile int temperature = 0;                     // 전역변수(Global Variable)
volatile int stop_temp = 0; // 정지 시 온도
volatile int state = GO;
volatile int buzz_state = OFF;
volatile int temp_int = 0;   //측정한 센서 값을 온도로 변환한 값, 정수만 저장.
volatile int num[4];

void init_twi_port();
int read_twi_2byte_nopreset(char reg);
void display_FND(int value);

ISR(TIMER0_OVF_vect) // Timer/Counter0 오버플로우 인터럽트 서비스 루틴
{
	TCNT0 = 17;                             // TCNT0 초기화
	if (buzz_state == ON)
	{
		PORTB |= 1 << 4;
		// 버저 포트 ON
	}
	else
	{
		PORTB &= ~(1 << 4);
		// 버저 포트 OFF
	}
}

int main(void) {
	int i;
	DDRC = 0xff; // C 포트는 FND 데이터 출력 신호
	DDRG = 0x0f; // G 포트는 FND 선택 출력 신호
	DDRE = 0xef; // 0b11101111, PE4(SW1)는 입력
	EICRB = 0x02; // INT4 트리거 모드는 하강 에지(falling edge)
	EIMSK = 0x10; // INT4 인터럽트 인애이블
	
	DDRB |= 0x10;                     // 버저 연결 포트(PB4) 출력 설정
	TCCR0 = 0x03;                     // 프리스케일러 32분주
	TIMSK = 0x01;                     // 오버플로우 인터럽트 활성화, TOIE0 비트 세트
	TCNT0 = 17;
	
	SREG |= 0x80; // SREG의 I(Interrupt Enalbe) 비트(bit7) ‘1’로 세트
	
	init_twi_port();                                                      // TWI 및 포트 초기화
	while (1)                                                         // 온도값 읽어 FND 디스플레이
	{
		for (i=0; i<60; i++)                                 // 0.6초 동안 처리
		{
			if (state == GO)                                // 온도계 모드인 경우
			{
				if ((i == 0) || (i == 30))              // 억세스시간(300ms) 동안 기다리기 // 위하여 30번에 1번 꼴로 실제 억세스
				temperature = read_twi_2byte_nopreset(LM75A_TEMP_REG);
				temp_int = 10*num[2] + num[1];
				if (temp_int >= 24 || temp_int < 20){
					buzz_state = ON;
				}
				else buzz_state = OFF;
				display_FND(temperature);        // 한 번 디스플레이에 약 10ms 소요
			}
			else                                                // 체온계 모드인 경우
			{
				temp_int = 10*num[2] + num[1];
				if (temp_int >= 30 || temp_int < 20){
					buzz_state = ON;
				}
				else buzz_state = OFF;
				if (i < 30)
				// 0.3초는 디스플레이
				{
					temperature = stop_temp;
					display_FND(temperature); //한 번 디스플레이에 약 10ms 소요
				}
				else
				// 0.3초는 디스플레이 끔
				{
					PORTG = 0x00;
					_delay_ms(10); // 10ms 딜레이
				}
			}
		}
	}
}

void init_twi_port() {
	TWSR = TWSR & 0xfc;
	// FND 출력 세팅
	// Prescaler 값 = 00 (1배)
	TWBR = (F_CPU/F_SCK - 16) / 2;
	// 공식 참조, bit rate 설정
}

int read_twi_2byte_nopreset(char reg)
{
	char high_byte, low_byte;
	TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN); // START 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08) ;
	// START 상태 검사, 이후 ACK 및 상태 검사
	TWDR = LM75A_ADDR | 0;
	// SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);
	// SLA+W 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18) ;
	TWDR = reg;                                                    // LM75A Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);                  // LM75A Reg 값 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
	TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN); // RESTART 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x10) ; //RESTART 상태 검사, 이후 ACK,NO_ACK 상태 검사
	TWDR = LM75A_ADDR | 1;                                                // SLA+R 준비, R=1
	TWCR = (1 << TWINT) | (1 << TWEN);                                 // SLA+R 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x40) ;
	TWCR = (1 << TWINT) | (1 << TWEN | 1 << TWEA);               // 1st DATA 준비
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x50) ;
	high_byte = TWDR; // 1st DATA 수신
	TWCR = (1 << TWINT) | (1 << TWEN); // 2nd DATA 준비
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x58) ;
	low_byte = TWDR;                                                           // 2nd DATA 수신
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);            // STOP 전송
	while ((TWCR & (1 << TWSTO))) ;                                        // STOP 확인
	return((high_byte<<8) | low_byte);                                      // 수신 DATA 리턴
}

void display_FND(int value)
{
	char digit[12] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x00, 0x40 }; // ‘0’~‘9’, ‘ ’, ‘-’
	char fnd_sel[4] = {0x01, 0x02, 0x04, 0x08};
	int value_int, value_deci, i;
	if ((value & 0x8000) != 0x8000)                            // Sign 비트 체크
	num[3] = 10;                                         // 양수인 경우는 디스플레이 없음
	else {
		num[3] = 11;
		value = (~value)+1; // 음수인 경우는 ‘-’ 디스플레이 // 2’s Complement 값을 취함
	}
	value_int = (char)((value & 0x7f00) >> 8);
	// High Byte bit6~0 값 (정수 값)
	value_deci = (char)(value & 0x0080);
	// Low Byte bit7 값 (소수 첫째자리값)
	num[2] = (value_int / 10) % 10;
	num[1] = value_int % 10;
	num[0] = (value_deci == 0x80) ? 5 : 0;                   // 소수 첫째자리가 1이면 0.5에 해당하므로 // 5를 디스플레이
	for(i=0; i<4; i++)
	{
		PORTC = digit[num[i]]; PORTG = fnd_sel[i]; if (i==1)
		PORTC |= 0x80;
		// 왼쪽에서 3번째 FND에는 소수점(.)을 찍음
		if (i%2)
		_delay_ms(2);
		// 2번은 2ms 지연
		else
		_delay_ms(3);
		// 2번은 3ms 지연, 총 10ms 지연
	}
}

ISR(INT4_vect)
{
	_delay_ms(100);                                  // 스위치 바운스 시간 동안 기다림
	EIFR |= 1 << 4;                                  // 바운스에 의해 생긴 인터럽트는 무효화 - 디바운싱
	if ((PINE & 0x10) != 0x00)                     // SW1이 눌려진 것이 아니면
	return;
	if (state == STOP)
	state = GO; // STOP 상태라면 GO 상태로 변경
	else
	{
		state = STOP;
		// GO 상태라면 STOP 상태로 변경
		stop_temp = read_twi_2byte_nopreset(LM75A_TEMP_REG);
		// 그리고, ‘현재 온도’를 읽어서 저장
	}
}