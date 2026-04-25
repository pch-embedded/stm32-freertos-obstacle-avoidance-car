# STM32 FreeRTOS Obstacle Avoidance Car

STM32H753ZI와 FreeRTOS를 기반으로 구현한 초음파 센서 장애물 감지 및 회피 주행 프로젝트입니다.

전방 장애물을 감지하면 차량을 정지시키고, 서보모터를 이용해 좌/우 방향을 스캔한 뒤 더 넓은 방향으로 회피 주행하도록 구현했습니다.  
단순한 센서-모터 연결이 아니라, 센서 입력, 제어 판단, 모터 구동을 FreeRTOS Task로 분리하고 Queue, Mutex, EventFlags, 상태머신을 적용하여 구조적인 임베디드 제어 흐름을 구현하는 것을 목표로 했습니다.

---

## 1. 프로젝트 개요

이 프로젝트는 초음파 센서 기반 장애물 감지와 모터 제어를 FreeRTOS 기반으로 분리 설계한 임베디드 주행 시스템입니다.

주요 기능은 다음과 같습니다.

- 전방 초음파 거리 측정
- 일정 거리 이하에서 자동 정지
- 서보모터를 이용한 중앙/좌/우 스캔
- 좌/우 거리 비교 후 회피 방향 결정
- 좌회전, 우회전, 후진 및 재탐색 동작
- FreeRTOS Task / Queue 기반 모터 명령 처리
- Mutex 기반 초음파 거리값 Snapshot 보호
- EventFlags 기반 Task 이벤트 동기화
- ControlTask 상태머신 기반 non-blocking 제어 구조
- 초음파 거리값 스파이크 필터링 적용

---

## 2. 개발 환경

- MCU: STM32H753ZI
- IDE: STM32CubeIDE
- Language: C
- RTOS: FreeRTOS
- Sensor: HC-SR04 Ultrasonic Sensor
- Servo Motor: SG90
- DC Motor Driver: L298N

---

## 3. 시스템 구성

### UltrasonicTrigTask

초음파 센서에 Trigger 펄스를 주기적으로 발생시킵니다.

### UltrasonicEdgeTask

Echo rising/falling edge를 Queue로 수신하고, Echo pulse 시간을 기반으로 거리값을 계산합니다.  
계산된 거리값은 Mutex로 보호되는 Snapshot 구조체에 저장됩니다.

### ControlTask

초음파 Snapshot을 읽어 전방 장애물 여부를 판단하고, 주행/정지/스캔/회피 동작을 결정합니다.  
기존의 blocking delay 기반 흐름을 상태머신으로 개선하여 ControlTask가 장시간 멈추지 않도록 구성했습니다.

### MotorTask

`motorCmdQ`를 통해 전달받은 모터 명령만 수행합니다.  
전진, 정지, 후진, 좌회전, 우회전 동작은 MotorTask가 전담합니다.

### ButtonTask

버튼 입력을 디바운싱한 뒤 EventFlags를 통해 ControlTask에 RUN/STOP 토글 이벤트를 전달합니다.

---

## 4. 동작 로직

1. 평소에는 중앙 방향 거리값을 기준으로 직진
2. 전방 장애물이 일정 거리 이하로 들어오면 정지
3. 서보모터를 중앙 → 좌측 → 우측 방향으로 이동시키며 거리 측정
4. 좌/우 중 더 넓은 방향으로 회전
5. 중앙/좌/우 모두 위험하면 후진
6. 회전 또는 후진 후 다시 전방을 확인하고 주행 재개
7. 버튼 입력 시 RUN/STOP 상태 전환

---

## 5. 주요 구현 내용

### FreeRTOS Task 분리

센서 입력, 제어 판단, 모터 구동을 각각 독립된 Task로 분리했습니다.

- UltrasonicTrigTask: 초음파 Trigger 발생
- UltrasonicEdgeTask: Echo edge 처리 및 거리 계산
- ControlTask: 주행 상태 판단 및 회피 로직 수행
- MotorTask: 모터 명령 실행
- ButtonTask: 버튼 입력 처리

### Queue 기반 모터 명령 처리

ControlTask가 모터를 직접 제어하지 않고, `MotorCmd_t` 명령을 `motorCmdQ`에 전달하도록 구성했습니다.  
MotorTask는 Queue에서 명령을 수신한 뒤 실제 GPIO/PWM 제어를 수행합니다.

지원하는 모터 명령은 다음과 같습니다.

- `MOTOR_CMD_RUN`
- `MOTOR_CMD_STOP`
- `MOTOR_CMD_BACK`
- `MOTOR_CMD_TURN_LEFT`
- `MOTOR_CMD_TURN_RIGHT`

### Mutex 기반 초음파 Snapshot 보호

초음파 거리값을 여러 Task가 직접 공유하지 않도록 `UsSnapshot_t` 구조체로 묶었습니다.

```c
typedef struct {
    uint32_t dist_cm;
    uint8_t  is_near;
    uint32_t pulse_us;
    uint32_t seq;
} UsSnapshot_t;
