/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 3-DOF Arm + 4WD Smooth + 2-Sensor Auto (PB12 Kiri, PB14 Kanan)
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*      1. KALIBRASI GRIPPER PA6 (TETAP & AMAN)                               */
/* ========================================================================== */
#define SAFE_OPEN_DEG       20    // Buka lebar (20 deg)
#define SAFE_CLOSE_DEG      42    // Titup Aman (42 deg)
#define ABSOLUTE_MIN_DEG    15    // Batas aman bawah
#define ABSOLUTE_MAX_DEG    58    // Batas aman atas

#define MICRO_STEP_US       2     // Melangkah halus per 2 mikrodetik
#define MICRO_DELAY_CLOSE   12    // Kecepatan menutup
#define MICRO_DELAY_OPEN    16    // Kecepatan membuka ekstra pelan
#define DETACH_DELAY_MS     80    // 80ms setelah sampai langsung putus PWM

/* ========================================================================== */
/*      2. LINK 1: SHOULDER MG996R (PA7) & LINK 2: SIKU MG90S (PB8)           */
/* ========================================================================== */
#define SHOULDER_INIT_DEG   110
#define SHOULDER_MIN_DEG    10
#define SHOULDER_MAX_DEG    160
#define SHOULDER_STEP_DEG   4
#define SHOULDER_RAMP_MS    25

#define ELBOW_INIT_DEG      105
#define ELBOW_MIN_DEG       20
#define ELBOW_MAX_DEG       165
#define ELBOW_STEP_DEG      4
#define ELBOW_RAMP_MS       22

/* ========================================================================== */
/*      3. DRIVER MOTOR DC 12V (SOFT, ANTI-JENGAT & TIDAK OVER)               */
/* ========================================================================== */
#define IN1_PORT            GPIOA
#define IN1_PIN             GPIO_PIN_2   // PA2
#define IN2_PORT            GPIOA
#define IN2_PIN             GPIO_PIN_3   // PA3
#define IN3_PORT            GPIOB
#define IN3_PIN             GPIO_PIN_0   // PB0
#define IN4_PORT            GPIOB
#define IN4_PIN             GPIO_PIN_1   // PB1

// Kecepatan dibuat kalem & halus (Range PWM: 0 - 999)
#define SPEED_START_MIN     270   // Torsi awal mulai bergerak (agar tidak njengat)
#define SPEED_NORMAL        460   // Kecepatan Maju/Mundur Manual (Halus & Enak)
#define SPEED_TURN          430   // Kecepatan Belok Kiri/Kanan Manual
#define SPEED_AUTO_FWD      380   // Kecepatan Autonomous (Ekstra pelan anti-kebablasan)

// Parameter Akselerasi Halus (Soft-Start Ramp)
#define RAMP_STEP_PWM       12    // Kenaikan PWM per step akselerasi
#define RAMP_INTERVAL_MS    15    // Interval waktu kenaikan PWM (ms)

/* ========================================================================== */
/*      4. SENSOR LINE FOLLOWER (HANYA 2 SENSOR: KIRI PB12 & KANAN PB14)      */
/* ========================================================================== */
#define LF_PORT             GPIOB
#define LF_LEFT_PIN         GPIO_PIN_12  // PB12 (Sensor Kiri)
#define LF_RIGHT_PIN        GPIO_PIN_14  // PB14 (Sensor Kanan)

TIM_HandleTypeDef htim2; // Motor DC PWM (PA0, PA1)
TIM_HandleTypeDef htim3; // Gripper (PA6), Shoulder (PA7)
TIM_HandleTypeDef htim4; // Siku PB8 (TIM4_CH3)
UART_HandleTypeDef huart1;

uint8_t rx_byte = 0;
volatile char pending_cmd = 0;
volatile uint8_t cmd_ready = 0;

/* Variabel Gripper */
uint8_t target_open    = SAFE_OPEN_DEG;   // 20 deg
uint8_t target_close   = SAFE_CLOSE_DEG;  // 42 deg
uint8_t current_angle  = SAFE_OPEN_DEG;
uint16_t current_pulse = 722;             // 20 deg = 722 us

/* Variabel Lengan */
uint8_t shoulder_angle = SHOULDER_INIT_DEG;
uint8_t elbow_angle    = ELBOW_INIT_DEG;

uint8_t servo_is_active = 0;
uint32_t servo_detach_timestamp = 0;

/* Variabel Roda Soft-Ramp (Anti-Jengat) */
uint16_t motor_speed       = SPEED_NORMAL;
uint16_t motor_turn_speed  = SPEED_TURN;
uint16_t current_pwm_a     = 0;
uint16_t current_pwm_b     = 0;
uint16_t target_pwm_a      = 0;
uint16_t target_pwm_b      = 0;
uint8_t  current_wheel_dir = 0; // 0:Stop, 1:Fwd, 2:Bwd, 3:Left, 4:Right
uint32_t last_ramp_tick    = 0;

/* Variabel Mode Autonomous (2 Sensor) */
uint8_t auto_mode = 0;                    // 0 = Manual, 1 = Autonomous Aktif
GPIO_PinState black_logic = GPIO_PIN_SET; // Default: HIGH (1) saat kena Hitam (tekan 'L' untuk balik)
uint8_t last_auto_state = 255;
uint8_t is_moving_auto = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

static void uart_send(const char *str) {
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);
}

static uint16_t deg_to_pulse(uint8_t deg) {
  if (deg > 180) deg = 180;
  return (uint16_t)(500 + ((uint32_t)deg * 2000 / 180));
}

static void detach_servo_pwm(void) {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  servo_is_active = 0;
}

/* ================= FUNGSI GRIPPER MICRO-STEPPING ================= */
static void move_gripper_micro_slow(uint8_t goal_angle) {
  if (goal_angle > ABSOLUTE_MAX_DEG) goal_angle = ABSOLUTE_MAX_DEG;
  if (goal_angle < ABSOLUTE_MIN_DEG) goal_angle = ABSOLUTE_MIN_DEG;

  uint16_t goal_pulse = deg_to_pulse(goal_angle);
  if (current_pulse == goal_pulse) return;

  uint8_t is_opening = (goal_pulse < current_pulse) ? 1 : 0;
  uint8_t step_delay = is_opening ? MICRO_DELAY_OPEN : MICRO_DELAY_CLOSE;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse);
  HAL_Delay(20);

  while (current_pulse != goal_pulse) {
    if (current_pulse < goal_pulse) {
      if (goal_pulse - current_pulse < MICRO_STEP_US) current_pulse = goal_pulse;
      else current_pulse += MICRO_STEP_US;
    } else {
      if (current_pulse - goal_pulse < MICRO_STEP_US) current_pulse = goal_pulse;
      else current_pulse -= MICRO_STEP_US;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse);

    uint16_t diff = (current_pulse > goal_pulse) ? (current_pulse - goal_pulse) : (goal_pulse - current_pulse);
    if (diff < 25) HAL_Delay(step_delay + 8);
    else HAL_Delay(step_delay);
  }

  HAL_Delay(50);
  if (!is_opening && current_pulse > deg_to_pulse(ABSOLUTE_MIN_DEG) + 10) {
    current_pulse -= 6;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse);
  } else if (is_opening && current_pulse < deg_to_pulse(ABSOLUTE_MAX_DEG) - 10) {
    current_pulse += 6;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse);
  }

  current_angle = goal_angle;
  servo_is_active = 1;
  servo_detach_timestamp = HAL_GetTick();
}

/* ================= GERAKAN SHOULDER & SIKU ================= */
static void move_shoulder_soft(uint8_t goal_angle) {
  if (goal_angle > SHOULDER_MAX_DEG) goal_angle = SHOULDER_MAX_DEG;
  if (goal_angle < SHOULDER_MIN_DEG) goal_angle = SHOULDER_MIN_DEG;

  while (shoulder_angle != goal_angle) {
    if (shoulder_angle < goal_angle) shoulder_angle++;
    else shoulder_angle--;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, deg_to_pulse(shoulder_angle));

    uint8_t diff = (shoulder_angle > goal_angle) ? (shoulder_angle - goal_angle) : (goal_angle - shoulder_angle);
    HAL_Delay((diff <= 2) ? (SHOULDER_RAMP_MS + 10) : SHOULDER_RAMP_MS);
  }
}

static void move_elbow_soft(uint8_t goal_angle) {
  if (goal_angle > ELBOW_MAX_DEG) goal_angle = ELBOW_MAX_DEG;
  if (goal_angle < ELBOW_MIN_DEG) goal_angle = ELBOW_MIN_DEG;

  while (elbow_angle != goal_angle) {
    if (elbow_angle < goal_angle) elbow_angle++;
    else elbow_angle--;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, deg_to_pulse(elbow_angle));

    uint8_t diff = (elbow_angle > goal_angle) ? (elbow_angle - goal_angle) : (goal_angle - elbow_angle);
    HAL_Delay((diff <= 2) ? (ELBOW_RAMP_MS + 10) : ELBOW_RAMP_MS);
  }
}

static void arm_soft_startup(void) {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, deg_to_pulse(shoulder_angle));
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(300);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, deg_to_pulse(elbow_angle));
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_Delay(300);

  current_angle = SAFE_OPEN_DEG;
  current_pulse = deg_to_pulse(SAFE_OPEN_DEG);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_pulse);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_Delay(150);
  detach_servo_pwm();
}

/* ================= GERAKAN 4 RODA 12V SOFT RAMP (ANTI-JENGAT) ================= */
static void motor_apply_pwm(uint16_t speed_a, uint16_t speed_b) {
  if (speed_a > 999) speed_a = 999;
  if (speed_b > 999) speed_b = 999;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_a);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed_b);
}

static void update_wheels_soft_ramp(void) {
  if (HAL_GetTick() - last_ramp_tick < RAMP_INTERVAL_MS) return;
  last_ramp_tick = HAL_GetTick();

  // Ramp Kanal A (Kiri)
  if (current_pwm_a < target_pwm_a) {
    if (current_pwm_a < SPEED_START_MIN) current_pwm_a = SPEED_START_MIN;
    else if (target_pwm_a - current_pwm_a <= RAMP_STEP_PWM) current_pwm_a = target_pwm_a;
    else current_pwm_a += RAMP_STEP_PWM;
  } else if (current_pwm_a > target_pwm_a) {
    current_pwm_a = target_pwm_a;
  }

  // Ramp Kanal B (Kanan)
  if (current_pwm_b < target_pwm_b) {
    if (current_pwm_b < SPEED_START_MIN) current_pwm_b = SPEED_START_MIN;
    else if (target_pwm_b - current_pwm_b <= RAMP_STEP_PWM) current_pwm_b = target_pwm_b;
    else current_pwm_b += RAMP_STEP_PWM;
  } else if (current_pwm_b > target_pwm_b) {
    current_pwm_b = target_pwm_b;
  }

  motor_apply_pwm(current_pwm_a, current_pwm_b);
}

static void wheels_set_motion_soft(uint8_t dir, uint16_t goal_a, uint16_t goal_b) {
  if (current_wheel_dir != dir) {
    current_wheel_dir = dir;
    current_pwm_a = SPEED_START_MIN;
    current_pwm_b = SPEED_START_MIN;
    motor_apply_pwm(current_pwm_a, current_pwm_b);
    last_ramp_tick = HAL_GetTick();
  }
  target_pwm_a = goal_a;
  target_pwm_b = goal_b;
}

static void wheels_stop(void) {
  target_pwm_a = 0;
  target_pwm_b = 0;
  current_pwm_a = 0;
  current_pwm_b = 0;
  current_wheel_dir = 0;
  motor_apply_pwm(0, 0);
  HAL_GPIO_WritePin(GPIOA, IN1_PIN | IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, IN3_PIN | IN4_PIN, GPIO_PIN_RESET);
  is_moving_auto = 0;
}

// Rem halus namun presisi saat mendeteksi putih
static void wheels_soft_brake_and_stop(void) {
  if (is_moving_auto) {
    HAL_GPIO_WritePin(GPIOA, IN1_PIN | IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, IN3_PIN | IN4_PIN, GPIO_PIN_SET);
    motor_apply_pwm(850, 850);
    HAL_Delay(45);
  }
  wheels_stop();
}

static void wheels_forward_custom(uint16_t spd_left, uint16_t spd_right) {
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
  wheels_set_motion_soft(1, spd_left, spd_right);
}

static void wheels_forward(void) {
  wheels_forward_custom(motor_speed, motor_speed);
}

static void wheels_backward(void) {
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
  wheels_set_motion_soft(2, motor_speed, motor_speed);
}

static void wheels_turn_left_custom(uint16_t spd) {
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
  wheels_set_motion_soft(3, spd, spd);
}

static void wheels_turn_right_custom(uint16_t spd) {
  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
  wheels_set_motion_soft(4, spd, spd);
}

/* ================= LOGIKA 2 SENSOR (PB12 KIRI & PB14 KANAN) ================= */
static uint8_t is_black(uint16_t pin) {
  return (HAL_GPIO_ReadPin(LF_PORT, pin) == black_logic) ? 1 : 0;
}

static void run_autonomous_line_follower(void) {
  uint8_t s_left  = is_black(LF_LEFT_PIN);  // PB12 (Kiri)
  uint8_t s_right = is_black(LF_RIGHT_PIN); // PB14 (Kanan)

  // Jika salah satu sensor (Kiri ATAU Kanan) mendeteksi PUTIH (0)
  if (s_left == 0 || s_right == 0) {
    // Verifikasi cepat 2ms (Debouncing) untuk memastikan benar-benar garis putih, bukan noise
    HAL_Delay(2);
    s_left  = is_black(LF_LEFT_PIN);
    s_right = is_black(LF_RIGHT_PIN);

    if (s_left == 0 || s_right == 0) {
      wheels_soft_brake_and_stop();
      auto_mode = 0; // OTOMATIS BERHENTI & KEMBALI KE MODE MANUAL!
      last_auto_state = 0;

      if (s_left == 0 && s_right == 0) {
        uart_send(">> [STOP PUTIH!] Kiri & Kanan Putih -> Rem & Kembali ke Mode MANUAL!\r\n");
      } else if (s_left == 0) {
        uart_send(">> [STOP PUTIH!] Sensor Kiri (PB12) Putih -> Rem & Kembali ke Mode MANUAL!\r\n");
      } else {
        uart_send(">> [STOP PUTIH!] Sensor Kanan (PB14) Putih -> Rem & Kembali ke Mode MANUAL!\r\n");
      }
      return;
    }
  }

  // Selama Kiri DAN Kanan masih Hitam (1 & 1) -> Jalan maju pelan & soft
  wheels_forward_custom(SPEED_AUTO_FWD, SPEED_AUTO_FWD);
  is_moving_auto = 1;
  if (last_auto_state != 1) {
    uart_send(">> [AUTO JALAN PELAN] Kiri & Kanan Hitam -> Merayap Soft (PWM: 380)\r\n");
    last_auto_state = 1;
  }
}

/* ================= UART CALLBACK ================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    char c = (char)rx_byte;
    if (c != '\r' && c != '\n' && c != ' ') {
      pending_cmd = c;
      cmd_ready = 1;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* Start PWM Roda 12V */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  wheels_stop();

  /* Jalankan Startup Aman Lengan */
  arm_soft_startup();

  uart_send("\r\n=== ROBOT ARM + 4WD SMOOTH (2 SENSOR: PB12 KIRI & PB14 KANAN) ===\r\n");
  uart_send("[AUTONOMOUS]   O: Start Auto (Putih=Off ke Manual) | V: Cek 2 Sensor | L: Polaritas\r\n");
  uart_send("[KECEPATAN]    1: Pelan (380) | 2: Normal Soft (460) | 3: Cepat (600)\r\n");
  uart_send("[GRIPPER PA6]  C: Buka (20)   | A: Tutup (42) | +/-: Tutup | N/M: Buka\r\n");
  uart_send("[ARM]          U/J: Shoulder (PA7) | I/K: Siku (PB8) | H: Home Atas\r\n");
  uart_send("[RODA MANUAL]  W: Maju Soft | S: Mundur Soft | Q: Kiri | D: Kanan | X: Stop\r\n");

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  char feedback[100];

  while (1) {
    if (cmd_ready) {
      cmd_ready = 0;
      char cmd = pending_cmd;

      /* ================= 1. KONTROL MODE AUTONOMOUS (2 SENSOR) ================= */
      if (cmd == 'O' || cmd == 'o') {
        if (!auto_mode) {
          // Cek dulu apakah saat ditekan 'O' posisi robot sudah di atas Hitam
          if (is_black(LF_LEFT_PIN) == 0 || is_black(LF_RIGHT_PIN) == 0) {
            uart_send(">> [PERINGATAN!] Sensor masih di atas PUTIH! Taruh di HITAM dulu baru tekan O.\r\n");
            wheels_stop();
            auto_mode = 0;
          } else {
            auto_mode = 1;
            last_auto_state = 255;
            uart_send(">> [AUTONOMOUS: AKTIF] Jalan Pelan, Kena Putih = Otomatis Jadi Manual!\r\n");
          }
        } else {
          auto_mode = 0;
          wheels_stop();
          uart_send(">> [AUTONOMOUS: MATI] Mode Manual Aktif.\r\n");
        }
      } else if (cmd == 'L' || cmd == 'l') {
        black_logic = (black_logic == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        last_auto_state = 255;
        sprintf(feedback, ">> [POLARITAS SENSOR] Hitam sekarang = %s\r\n",
                (black_logic == GPIO_PIN_SET) ? "HIGH (1)" : "LOW (0)");
        uart_send(feedback);
      } else if (cmd == 'V' || cmd == 'v') {
        sprintf(feedback,
                ">> [CEK 2 SENSOR] Kiri(PB12):%d | Kanan(PB14):%d (1=Hitam, 0=Putih)\r\n",
                is_black(LF_LEFT_PIN), is_black(LF_RIGHT_PIN));
        uart_send(feedback);
      }

      /* ================= PILIHAN KECEPATAN RODA (1, 2, 3) ================= */
      else if (cmd == '1') {
        motor_speed = 380;
        motor_turn_speed = 360;
        uart_send(">> [SPEED 1] Mode Ekstra Pelan (PWM: 380)\r\n");
      } else if (cmd == '2') {
        motor_speed = SPEED_NORMAL; // 460
        motor_turn_speed = SPEED_TURN; // 430
        uart_send(">> [SPEED 2] Mode Normal Soft (PWM: 460)\r\n");
      } else if (cmd == '3') {
        motor_speed = 600;
        motor_turn_speed = 550;
        uart_send(">> [SPEED 3] Mode Agak Cepat (PWM: 600)\r\n");
      }

      /* ================= 2. KONTROL GRIPPER SUPER PELAN ================= */
      else if (cmd == 'A' || cmd == 'a') {
        uart_send(">> Menutup pelan...\r\n");
        move_gripper_micro_slow(target_close);
        sprintf(feedback, ">> [A] TUTUP SELESAI: %d deg\r\n", current_angle);
        uart_send(feedback);
      } else if (cmd == 'C' || cmd == 'c') {
        sprintf(feedback, ">> Membuka pelan ke %d deg...\r\n", target_open);
        uart_send(feedback);
        move_gripper_micro_slow(target_open);
        sprintf(feedback, ">> [C] BUKA SELESAI: %d deg\r\n", current_angle);
        uart_send(feedback);
      } else if (cmd == '+') {
        if (target_close < ABSOLUTE_MAX_DEG) target_close += 1;
        move_gripper_micro_slow(target_close);
        sprintf(feedback, ">> [+] TUTUP +1: %d deg\r\n", target_close);
        uart_send(feedback);
      } else if (cmd == '-') {
        if (target_close > target_open + 2) target_close -= 1;
        move_gripper_micro_slow(target_close);
        sprintf(feedback, ">> [-] TUTUP -1: %d deg\r\n", target_close);
        uart_send(feedback);
      } else if (cmd == 'N' || cmd == 'n') {
        if (target_open < target_close - 2) target_open += 1;
        move_gripper_micro_slow(target_open);
        sprintf(feedback, ">> [N] BUKA +1: %d deg\r\n", target_open);
        uart_send(feedback);
      } else if (cmd == 'M' || cmd == 'm') {
        if (target_open > ABSOLUTE_MIN_DEG) target_open -= 1;
        move_gripper_micro_slow(target_open);
        sprintf(feedback, ">> [M] BUKA -1 (Makin Lebar): %d deg\r\n", target_open);
        uart_send(feedback);
      } else if (cmd == 'P' || cmd == 'p') {
        detach_servo_pwm();
        uart_send(">> PWM Gripper Dimatikan\r\n");
      }

      /* ================= 3. KONTROL SHOULDER & SIKU ================= */
      else if (cmd == 'U' || cmd == 'u') {
        uint8_t next = (shoulder_angle + SHOULDER_STEP_DEG <= SHOULDER_MAX_DEG)
                           ? (shoulder_angle + SHOULDER_STEP_DEG)
                           : SHOULDER_MAX_DEG;
        move_shoulder_soft(next);
        sprintf(feedback, ">> [U] SHOULDER NAIK: %d deg\r\n", shoulder_angle);
        uart_send(feedback);
      } else if (cmd == 'J' || cmd == 'j') {
        uint8_t next = (shoulder_angle >= SHOULDER_MIN_DEG + SHOULDER_STEP_DEG)
                           ? (shoulder_angle - SHOULDER_STEP_DEG)
                           : SHOULDER_MIN_DEG;
        move_shoulder_soft(next);
        sprintf(feedback, ">> [J] SHOULDER NUNDUK: %d deg\r\n", shoulder_angle);
        uart_send(feedback);
      } else if (cmd == 'I' || cmd == 'i') {
        uint8_t next = (elbow_angle + ELBOW_STEP_DEG <= ELBOW_MAX_DEG)
                           ? (elbow_angle + ELBOW_STEP_DEG)
                           : ELBOW_MAX_DEG;
        move_elbow_soft(next);
        sprintf(feedback, ">> [I] SIKU NAIK: %d deg\r\n", elbow_angle);
        uart_send(feedback);
      } else if (cmd == 'K' || cmd == 'k') {
        uint8_t next = (elbow_angle >= ELBOW_MIN_DEG + ELBOW_STEP_DEG)
                           ? (elbow_angle - ELBOW_STEP_DEG)
                           : ELBOW_MIN_DEG;
        move_elbow_soft(next);
        sprintf(feedback, ">> [K] SIKU TURUN: %d deg\r\n", elbow_angle);
        uart_send(feedback);
      } else if (cmd == 'H' || cmd == 'h') {
        uart_send(">> [H] Mengembalikan Lengan ke Posisi Atas...\r\n");
        move_elbow_soft(ELBOW_INIT_DEG);
        move_shoulder_soft(SHOULDER_INIT_DEG);
        sprintf(feedback, ">> STANDBY ATAS -> Shoulder: %d, Siku: %d\r\n",
                shoulder_angle, elbow_angle);
        uart_send(feedback);
      }

      /* ================= 4. KONTROL RODA MANUAL (SOFT & ANTI-JENGAT) ================= */
      else if (cmd == 'W' || cmd == 'w') {
        auto_mode = 0;
        wheels_forward();
        uart_send(">> [MANUAL] MAJU SOFT\r\n");
      } else if (cmd == 'S' || cmd == 's') {
        auto_mode = 0;
        wheels_backward();
        uart_send(">> [MANUAL] MUNDUR SOFT\r\n");
      } else if (cmd == 'Q' || cmd == 'q') {
        auto_mode = 0;
        wheels_turn_left_custom(motor_turn_speed);
        uart_send(">> [MANUAL] BELOK KIRI SOFT\r\n");
      } else if (cmd == 'D' || cmd == 'd') {
        auto_mode = 0;
        wheels_turn_right_custom(motor_turn_speed);
        uart_send(">> [MANUAL] BELOK KANAN SOFT\r\n");
      } else if (cmd == 'X' || cmd == 'x') {
        auto_mode = 0;
        wheels_stop();
        uart_send(">> [MANUAL] STOP\r\n");
      }
    }

    /* Update Akselerasi Halus (Soft-Ramp) Setiap Loop */
    update_wheels_soft_ramp();

    /* Eksekusi Sensor dengan Respons 1ms saat Mode Autonomous Aktif */
    if (auto_mode) {
      run_autonomous_line_follower();
      HAL_Delay(1);
    } else {
      HAL_Delay(5);
    }

    /* Auto-Detach PWM Gripper */
    if (servo_is_active &&
        (HAL_GetTick() - servo_detach_timestamp >= DETACH_DELAY_MS)) {
      detach_servo_pwm();
    }
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_TIM3_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim3);
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_TIM4_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_PWM_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
}

static void MX_USART1_UART_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, IN1_PIN | IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, IN3_PIN | IN4_PIN, GPIO_PIN_RESET);

  /* LED PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* IN1 (PA2) & IN2 (PA3) Motor Kiri */
  GPIO_InitStruct.Pin = IN1_PIN | IN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* IN3 (PB0) & IN4 (PB1) Motor Kanan */
  GPIO_InitStruct.Pin = IN3_PIN | IN4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 2 SENSOR LINE FOLLOWER: Hanya PB12 (Kiri) & PB14 (Kanan) */
  GPIO_InitStruct.Pin = LF_LEFT_PIN | LF_RIGHT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UART1 PA9 (TX) & PA10 (RX) */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA0 (TIM2_CH1) & PA1 (TIM2_CH2) - ENA & ENB */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA6 (TIM3_CH1 - Gripper) & PA7 (TIM3_CH2 - Shoulder) */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB8 (TIM4_CH3 - Siku MG90S) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
