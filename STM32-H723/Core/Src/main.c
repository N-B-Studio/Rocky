/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "ws2812.h"
#include "bsp_fdcan.h"
#include "BMI088Middleware.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint8_t node_id;
    float   cmd_deg;
} MotorAxis;

MotorAxis motor_left_wheel   = {1, 0};
MotorAxis motor_right_wheel  = {2, 0};
MotorAxis motor_neck         = {3, 0};  // IMU\u6302\u5728\u8116\u5b50\u4e0a



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 1kHz flag
volatile uint8_t loop_1khz_flag = 0;

// attitude
static float pitch_deg = 0.0f;  // \u55ac\u4eea\u89d2-\u7528\u4e8e\u5e73\u8861\u63a7\u5236

// target command (deg) - \u53ea\u63a7\u5236pitch\u7528\u4e8eLR\u8f6e\u5e73\u8861
static float cmd_pitch_deg = 0.0f;

// gains
static float K_pitch = 1.0f;  // \u5e73\u8861\u53cd\u9988\u589e\u76ca

// dt
static const float dt = 0.001f;
static const float RAD2DEG = 57.2957795f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===== WS2812 LED status helpers =====
static inline void led_yellow(void){ WS2812_Ctrl(60, 60, 0); }
static inline void led_red(void)   { WS2812_Ctrl(80, 0, 0);  }
static inline void led_green(void) { WS2812_Ctrl(0, 80, 0);  }

// ===== UART1 logging =====
static char logbuf[160];

static void log_uart1(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static void logf_uart1(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    // 注意：vsnprintf 返回“本应写入的长度”，可能 >= sizeof(logbuf)
    int n = vsnprintf(logbuf, sizeof(logbuf), fmt, ap);

    va_end(ap);

    if (n < 0) return;

    // 实际可发送长度 = buffer 内真实字符串长度（避免发到旧内容）
    size_t len = strnlen(logbuf, sizeof(logbuf));

    HAL_UART_Transmit(&huart1, (uint8_t*)logbuf, (uint16_t)len, 100);
}


// ===== Gimbal enable: PC14 =====
#define GIMBAL_EN_PORT GPIOC
#define GIMBAL_EN_PIN  GPIO_PIN_14

static inline void gimbal_enable(uint8_t en)
{
    HAL_GPIO_WritePin(GIMBAL_EN_PORT, GIMBAL_EN_PIN, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ===== Nodes =====
#define NODE_LEFT_WHEEL  1  // 左轮 - 平衡控制
#define NODE_RIGHT_WHEEL 2  // 右轮 - 平衡控制
#define NODE_NECK        3  // 脖子 - 固定零位（IMU挂在脖子上）

// ===== JC SDO write helpers =====
static inline uint16_t jc_tx_id(uint8_t node_id){ return (uint16_t)(0x600 + node_id); }

static inline void be_u16(uint8_t *p, uint16_t v){
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}
static inline void be_i32(uint8_t *p, int32_t v){
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v & 0xFF);
}

static int jc_enter_closed_loop(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 2B 00 A2 00 00 01 00 00
    uint8_t d[8] = {0x2B, 0x00, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

// mode: 4 = 位置直通 
static int jc_set_mode(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t mode)
{
    // 2B 00 60 00 00 mm 00 00
    uint8_t d[8] = {0x2B, 0x00, 0x60, 0x00, 0, 0, 0, 0};
    be_u16(&d[4], mode);
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_set_abs_pos_deg(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float deg)
{
    // 23 00 23 00 [int32_be] , deg*100
    int32_t v = (int32_t)(deg * 100.0f);
    uint8_t d[8] = {0x23, 0x00, 0x23, 0x00, 0, 0, 0, 0};
    be_i32(&d[4], v);
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

// 0=力矩模式，4=位置直通模式
static int jc_set_torque_nm(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float torque_nm)
{
    // 文档：扭矩寄存器 0x0020，放大 100 倍
    int16_t v = (int16_t)(torque_nm * 100.0f);

    uint8_t d[8] = {0x2B, 0x00, 0x20, 0x00, 0, 0, 0, 0};

    d[4] = (uint8_t)((v >> 8) & 0xFF);
    d[5] = (uint8_t)(v & 0xFF);

    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}
static int jc_set_speed_rpm(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float rpm)
{
    // 文档：速度寄存器 0x0021，rpm * 100
    int32_t v = (int32_t)(rpm * 100.0f);

    uint8_t d[8] = {0x23, 0x00, 0x21, 0x00, 0, 0, 0, 0};
    be_i32(&d[4], v);

    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_set_low_speed_rpm(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float rpm)
{
    // 文档：低速寄存器 0x0027，rpm * 100，2字节
    int16_t v = (int16_t)(rpm * 100.0f);

    uint8_t d[8] = {0x2B, 0x00, 0x27, 0x00, 0, 0, 0, 0};

    d[4] = (uint8_t)((v >> 8) & 0xFF);
    d[5] = (uint8_t)(v & 0xFF);

    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}
// 读模式寄存器，测试用
static int jc_read_mode(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 读控制模式寄存器 0x0060
    uint8_t d[8] = {0x4B, 0x00, 0x60, 0x00, 0, 0, 0, 0};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}
static int jc_idle(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 2B 00 A0 00 00 01 00 00
    uint8_t d[8] = {0x2B, 0x00, 0xA0, 0x00, 0x00, 0x01, 0x00, 0x00};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_read_speed(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 读实时速度 0x0006
    uint8_t d[8] = {0x43, 0x00, 0x06, 0x00, 0, 0, 0, 0};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_read_position(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 读实时位置 0x0008
    uint8_t d[8] = {0x43, 0x00, 0x08, 0x00, 0, 0, 0, 0};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_read_current(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 读母线电流 0x0005
    uint8_t d[8] = {0x4B, 0x00, 0x05, 0x00, 0, 0, 0, 0};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static int jc_read_error(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    // 读错误码 0x000C
    uint8_t d[8] = {0x43, 0x00, 0x0C, 0x00, 0, 0, 0, 0};
    return fdcanx_send_data(hcan, jc_tx_id(node_id), d, 8);
}

static void torque_step_test(uint8_t node_id, float tq, uint32_t hold_ms)
{
    logf_uart1("[TQ] node=%d tq=%.2fNm hold=%lums\r\n", node_id, tq, hold_ms);

    jc_set_torque_nm(&hfdcan1, node_id, tq);

    uint32_t t0 = HAL_GetTick();
    uint32_t last_read = 0;

    while ((HAL_GetTick() - t0) < hold_ms)
    {
        uint32_t now = HAL_GetTick();

        // 每100ms请求一次状态。需要你用Canable/candump看0x581回复
        if ((now - last_read) >= 100)
        {
            last_read = now;

            jc_read_speed(&hfdcan1, node_id);
            HAL_Delay(2);

            jc_read_position(&hfdcan1, node_id);
            HAL_Delay(2);

            jc_read_current(&hfdcan1, node_id);
            HAL_Delay(2);

            jc_read_error(&hfdcan1, node_id);
            HAL_Delay(2);
        }

        HAL_Delay(1);
    }

    jc_set_torque_nm(&hfdcan1, node_id, 0.0f);
    HAL_Delay(500);
}
static void fatal(const char *msg)
{
    led_red();
    logf_uart1("[FATAL] %s\r\n", msg);
    gimbal_enable(0);
    __disable_irq();
    while (1) { HAL_Delay(1000); }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
  led_yellow();
  log_uart1("\r\n[BOOT] H725 gimbal IMU sync control\r\n");

  // power off first
  gimbal_enable(0);
  HAL_Delay(50);

  // CAN
  log_uart1("[INIT] CAN...\r\n");
  bsp_can_init();
  HAL_Delay(50);

  // IMU
  log_uart1("[INIT] IMU...\r\n");
  uint8_t imu_err = BMI088_init();
  if (imu_err) fatal("BMI088_init failed");

  // 1kHz timer
  HAL_TIM_Base_Start_IT(&htim2);

  // enable gimbal power
  log_uart1("[INIT] GIMBAL EN=1\r\n");
  gimbal_enable(1);
  HAL_Delay(500);



  // node1/2: torque mode for balance wheels
  /* node3: position passthrough for head
  if (jc_set_mode(&hfdcan1, NODE_LEFT_WHEEL,  0) != 0) fatal("torque mode node1");
  HAL_Delay(100);

  if (jc_set_mode(&hfdcan1, NODE_RIGHT_WHEEL, 0) != 0) fatal("torque mode node2");
  HAL_Delay(100);

  if (jc_set_mode(&hfdcan1, NODE_NECK,        4) != 0) fatal("position mode node3");
  HAL_Delay(100);

  // closed loop + mode=4 (position passthrough)
  log_uart1("[INIT] node1/2 torque mode, node3 position mode\r\n");
  if (jc_enter_closed_loop(&hfdcan1, NODE_LEFT_WHEEL)  != 0) fatal("closed_loop node1");
  HAL_Delay(20);
  if (jc_enter_closed_loop(&hfdcan1, NODE_RIGHT_WHEEL) != 0) fatal("closed_loop node2");
  HAL_Delay(20);
  if (jc_enter_closed_loop(&hfdcan1, NODE_NECK)        != 0) fatal("closed_loop node3");
  HAL_Delay(20);
  // start at 0 deg - 脖子固定在零位
  //jc_set_abs_pos_deg(&hfdcan1, NODE_NECK, 0.0f);
  HAL_Delay(20);
  log_uart1("[INIT] neck initialized to 0deg\r\n");

  led_green();
  log_uart1("[OK] running 1kHz IMU->CAN\r\n");
  
  jc_read_mode(&hfdcan1, NODE_LEFT_WHEEL);
  HAL_Delay(50);

  jc_read_mode(&hfdcan1, NODE_RIGHT_WHEEL);
  HAL_Delay(50);

  jc_read_mode(&hfdcan1, NODE_NECK);
  HAL_Delay(50);

  */
  log_uart1("[INIT] single node torque mode test\r\n");

  // 先让1号空闲
  jc_idle(&hfdcan1, 1);
  HAL_Delay(200);

  // 切到mode0力矩模式
  jc_set_mode(&hfdcan1, 1, 0);
  HAL_Delay(200);

  // 读一下模式。你需要用Canable/candump看0x581返回
  jc_read_mode(&hfdcan1, 1);
  HAL_Delay(100);

  // 进入闭环
  jc_enter_closed_loop(&hfdcan1, 1);
  HAL_Delay(300);

  // 再读错误、速度、位置、电流
  jc_read_error(&hfdcan1, 1);
  HAL_Delay(50);
  jc_read_speed(&hfdcan1, 1);
  HAL_Delay(50);
  jc_read_position(&hfdcan1, 1);
  HAL_Delay(50);
  jc_read_current(&hfdcan1, 1);
  HAL_Delay(50);

  log_uart1("[INIT] node1 mode0 closed-loop done\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_50ms_log_ms = HAL_GetTick();
  static const float KP = 0.03f;
  static const float KD = 0.008f;
  static const float KI = 0.0f;

  static const float MAX_TORQUE_NM = 0.8f;
  static const float MAX_TILT_DEG = 35.0f;

  static float pitch_base_deg = 0.0f;     // 上电时手扶直立校准
  static float gyro_bias_dps  = 0.0f;
  static float pitch_i = 0.0f;
  static uint8_t calibrated = 0;
  static uint16_t calib_cnt = 0;
  static float pitch_sum = 0.0f;
  static float gyro_sum = 0.0f;
  static uint8_t head_div = 0;

  float gyro[3], accel[3], temp = 0.0f;
  const float alpha = 0.98f;
log_uart1("[TEST] MODE0 torque ramp node1\r\n");

while (1)
{
    // 先确保还是mode0
    jc_idle(&hfdcan1, 1);
    HAL_Delay(200);

    jc_set_mode(&hfdcan1, 1, 0);
    HAL_Delay(200);

    jc_enter_closed_loop(&hfdcan1, 1);
    HAL_Delay(300);

    log_uart1("[PHASE A] positive torque ramp\r\n");

    torque_step_test(1, 0.05f, 1500);
    torque_step_test(1, 0.10f, 1500);
    torque_step_test(1, 0.15f, 1500);
    torque_step_test(1, 0.20f, 1500);
    torque_step_test(1, 0.30f, 1500);
    torque_step_test(1, 0.40f, 1500);
    torque_step_test(1, 0.50f, 1500);

    jc_set_torque_nm(&hfdcan1, 1, 0.0f);
    HAL_Delay(2000);

    log_uart1("[PHASE B] negative torque ramp\r\n");

    torque_step_test(1, -0.05f, 1500);
    torque_step_test(1, -0.10f, 1500);
    torque_step_test(1, -0.15f, 1500);
    torque_step_test(1, -0.20f, 1500);
    torque_step_test(1, -0.30f, 1500);
    torque_step_test(1, -0.40f, 1500);
    torque_step_test(1, -0.50f, 1500);

    jc_set_torque_nm(&hfdcan1, 1, 0.0f);
    HAL_Delay(3000);

   /*
    while (1)
  {
	  if (!loop_1khz_flag) continue;
	      loop_1khz_flag = 0;

	      // 1) read IMU
	      BMI088_read(gyro, accel, &temp);

	      // 假设 BMI088Middleware 输出 gyro单位=deg/s
	      float gx = gyro[0];
	      float gy = gyro[1];
	      float gz = gyro[2];

	      float ax = accel[0];
	      float ay = accel[1];
	      float az = accel[2];

      // 4) command = balance control based on pitch (stabilize)

      uint32_t ms = HAL_GetTick();

      float pitch_acc = atan2f(-ay, sqrtf(ax * ax + az * az)) * RAD2DEG;

      // 先假设 gx 是 pitch gyro
      float gyro_pitch = -gx;

      pitch_deg = alpha * (pitch_deg + gyro_pitch * dt)
                + (1.0f - alpha) * pitch_acc;

      // 上电手扶直立校准 1 秒
      if (!calibrated)
      {
          pitch_sum += pitch_deg;
          gyro_sum  += gyro_pitch;
          calib_cnt++;

          if (calib_cnt >= 1000)
          {
              pitch_base_deg = pitch_sum / 1000.0f;
              gyro_bias_dps  = gyro_sum  / 1000.0f;
              calibrated = 1;

              logf_uart1("[CAL] pitch_base=%.2f gyro_bias=%.2f\r\n",
                        pitch_base_deg, gyro_bias_dps);
          }

          jc_set_torque_nm(&hfdcan1, 1, 0.0f);
          jc_set_torque_nm(&hfdcan1, 2, 0.0f);
          continue;
      }

      float pitch_err = pitch_deg - pitch_base_deg;
      float gyro_rate = gyro_pitch - gyro_bias_dps;

      if (fabsf(pitch_err) > MAX_TILT_DEG)
      {
          pitch_i = 0.0f;

          jc_set_torque_nm(&hfdcan1, 1, 0.0f);
          jc_set_torque_nm(&hfdcan1, 2, 0.0f);
          continue;
      }

      pitch_i += pitch_err * dt;

      if (pitch_i > 20.0f) pitch_i = 20.0f;
      if (pitch_i < -20.0f) pitch_i = -20.0f;

      float torque_cmd =
            KP * pitch_err
          + KD * gyro_rate
          + KI * pitch_i;

      if (torque_cmd >  MAX_TORQUE_NM) torque_cmd =  MAX_TORQUE_NM;
      if (torque_cmd < -MAX_TORQUE_NM) torque_cmd = -MAX_TORQUE_NM;

      // 你说轮子方向是反的，所以这里先反号
      int r1 = jc_set_torque_nm(&hfdcan1, 1, -torque_cmd);
      int r2 = jc_set_torque_nm(&hfdcan1, 2,  torque_cmd);

      // 3号头部 50Hz
      head_div++;
      if (head_div >= 20)
      {
          head_div = 0;
          float head_pos = 15.0f * sinf(ms * 0.001f * 0.5f);
          jc_set_abs_pos_deg(&hfdcan1, 3, head_pos);
      }

      uint32_t now = HAL_GetTick();
      if ((now - last_50ms_log_ms) >= 100)
      {
          last_50ms_log_ms = now;
          logf_uart1(
            "ACC %.3f %.3f %.3f | GYRO %.3f %.3f %.3f | Pacc %.2f P %.2f E %.2f Gr %.2f T %.3f r %d %d\r\n",
            ax, ay, az,
            gx, gy, gz,
            pitch_acc, pitch_deg, pitch_err, gyro_rate,
            torque_cmd, r1, r2
          );
      }
          
      */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        loop_1khz_flag = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
