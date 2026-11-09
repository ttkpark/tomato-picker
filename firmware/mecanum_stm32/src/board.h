// board.h — 기판(hardware_design/mecanum_stm32_pcb/gen_pcb.py)과 1:1. 핀을 바꾸면 기판도 바뀐다.
#pragma once

// 모터 = DRV8874 ×4, PWM 모드(IN1/IN2): IN1=PWM·IN2=0 → 정방향 구동/코스트. 바퀴 0=A/RR 1=B/RL 2=C/FR 3=D/FL
//   A  TIM8  CH1 PC6 / CH2 PC7      AF3
//   B  TIM8  CH3 PC8 / CH4 PC9      AF3
//   C  TIM9  CH1 PA2 / CH2 PA3      AF3
//   D  TIM12 CH1 PB14 / CH2 PB15    AF9
// 엔코더(타이머 엔코더 모드, 4체배): A=TIM2 PA15/PB3(AF1, 32비트) B=TIM3 PB4/PB5(AF2) C=TIM4 PB6/PB7(AF2)
//                                    D=TIM5 PA0/PA1(AF2, 32비트)
// nFAULT: PC10 PC11 PC12 PD2 (드라이버 오픈드레인, 기판 풀업 + 내부 풀업)
// IPROPI: PC0..PC3 = ADC1 IN10..IN13 (1kΩ → 0.45V/A)
// 배터리 PC4 = IN14 (100k/10k, 역접속 FET 뒤·릴레이 앞) · VM PC5 = IN15 (릴레이 뒤) · 5V PB0 = IN8 (10k/10k)
// 3V3(VDDA) = 내부 VREFINT(IN17)와 공장 보정값으로 잰다
// nSLEEP PB12 (4개 공통, 기판 풀다운) · E-stop 감지 PB13 (HIGH = 릴레이 코일 붙음 = 모터 허용, 100k 풀다운)
// LED PB11 하트비트 · PB1 폴트 · 링크 USART1 PA9 TX / PA10 RX (AF7) → ADuM1201 → CH340C
// SWD PA13/PA14는 건드리지 않는다. PA15/PB3/PB4(JTAG)는 AF만 바꿔 엔코더로 쓴다(F4엔 리맵이 없다).

#define HSE_HZ 8000000u
#define SYSCLK_HZ 168000000u
#define PCLK1_HZ 42000000u        // APB1 /4 → 타이머 84MHz
#define PCLK2_HZ 84000000u        // APB2 /2 → 타이머 168MHz
#define LINK_BAUD 115200u

// 설정 섹터 — 펌웨어 바이너리(섹터 0~9, 768KB) 밖. stm32flash는 바이너리가 덮는 섹터만 지우므로 굽기가 설정을 안 지운다.
#define CFG_SEC_A 10u
#define CFG_SEC_B 11u
#define CFG_ADDR_A 0x080C0000u
#define CFG_ADDR_B 0x080E0000u
#define CFG_SEC_SIZE 0x20000u
