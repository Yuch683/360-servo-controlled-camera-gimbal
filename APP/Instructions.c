#include "Instructions.h"

#include "Servo_Contral.h"
#include "Input_Solution.h"
#include "usart.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CMD_BUFFER_SIZE 96U
#define TRACK_PACKET_LEN 8U
#define TRACK_PACKET_HEAD1 0xFFU
#define TRACK_PACKET_HEAD2 0xAAU
#define TRACK_PACKET_TAIL  0x55U

typedef enum
{
  PACKET_WAIT_HEAD1 = 0,
  PACKET_WAIT_HEAD2,
  PACKET_READ_XH,
  PACKET_READ_XL,
  PACKET_READ_YH,
  PACKET_READ_YL,
  PACKET_READ_CHECKSUM,
  PACKET_READ_TAIL
} PacketParseState;

static volatile uint8_t g_cmd_ready = 0U;
static volatile uint8_t g_cmd_overflow = 0U;
static volatile uint16_t g_cmd_len = 0U;
static char g_cmd_buffer[CMD_BUFFER_SIZE];
static volatile uint8_t g_packet_ready = 0U;
static volatile uint16_t g_packet_checksum_error = 0U;
static volatile int16_t g_packet_x = 0;
static volatile int16_t g_packet_y = 0;
static PacketParseState g_packet_state = PACKET_WAIT_HEAD1;
static uint8_t g_packet_buf[TRACK_PACKET_LEN];

static void UART1_SendString(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), 100U);
}

static void UART1_SendFormat(const char *fmt, ...)
{
  char tx_buf[128];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if ((uint32_t)len >= sizeof(tx_buf))
  {
    len = (int)sizeof(tx_buf) - 1;
    tx_buf[len] = '\0';
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 100U);
}

static char *TrimLeft(char *str)
{
  while ((*str != '\0') && isspace((unsigned char)*str))
  {
    str++;
  }

  return str;
}

static void ToUppercase(char *str)
{
  while (*str != '\0')
  {
    *str = (char)toupper((unsigned char)*str);
    str++;
  }
}

//上位机二进制数据转化
static uint8_t TrackPacket_ParseByte(uint8_t rx_byte) 
{
  uint8_t checksum;
  uint16_t raw_x;
  uint16_t raw_y;

  switch (g_packet_state)
  {
    case PACKET_WAIT_HEAD1:
      if (rx_byte == TRACK_PACKET_HEAD1)
      {
        g_packet_buf[0] = rx_byte;
        g_packet_state = PACKET_WAIT_HEAD2;
        return 1U;
      }
      return 0U;

    case PACKET_WAIT_HEAD2:
      if (rx_byte == TRACK_PACKET_HEAD2)
      {
        g_packet_buf[1] = rx_byte;
        g_packet_state = PACKET_READ_XH;
      }
      else if (rx_byte == TRACK_PACKET_HEAD1)
      {
        g_packet_buf[0] = rx_byte;
        g_packet_state = PACKET_WAIT_HEAD2;
      }
      else
      {
        g_packet_state = PACKET_WAIT_HEAD1;
      }
      return 1U;

    case PACKET_READ_XH:
      g_packet_buf[2] = rx_byte;
      g_packet_state = PACKET_READ_XL;
      return 1U;

    case PACKET_READ_XL:
      g_packet_buf[3] = rx_byte;
      g_packet_state = PACKET_READ_YH;
      return 1U;

    case PACKET_READ_YH:
      g_packet_buf[4] = rx_byte;
      g_packet_state = PACKET_READ_YL;
      return 1U;

    case PACKET_READ_YL:
      g_packet_buf[5] = rx_byte;
      g_packet_state = PACKET_READ_CHECKSUM;
      return 1U;

    case PACKET_READ_CHECKSUM:
      g_packet_buf[6] = rx_byte;
      g_packet_state = PACKET_READ_TAIL;
      return 1U;

    case PACKET_READ_TAIL:
      g_packet_buf[7] = rx_byte;
      checksum = (uint8_t)(g_packet_buf[2] + g_packet_buf[3] + g_packet_buf[4] + g_packet_buf[5]);
      if ((g_packet_buf[7] == TRACK_PACKET_TAIL) && (checksum == g_packet_buf[6]))
      {
        if (g_packet_ready == 0U)
        {
          raw_x = ((uint16_t)g_packet_buf[2] << 8) | (uint16_t)g_packet_buf[3];
          raw_y = ((uint16_t)g_packet_buf[4] << 8) | (uint16_t)g_packet_buf[5];
          g_packet_x = (int16_t)raw_x;
          g_packet_y = (int16_t)raw_y;
          g_packet_ready = 1U;
        }
      }
      else
      {
        g_packet_checksum_error++;
      }

      g_packet_state = PACKET_WAIT_HEAD1;
      return 1U;

    default:
      g_packet_state = PACKET_WAIT_HEAD1;
      return 1U;
  }
}

void Instructions_Init(void)
{
  g_cmd_ready = 0U;
  g_cmd_overflow = 0U;
  g_cmd_len = 0U;
  g_packet_ready = 0U;
  g_packet_checksum_error = 0U;
  g_packet_state = PACKET_WAIT_HEAD1;

  UART1_SendString("\r\nUART command interface ready. Binary tracking packet enabled.\r\n");
}

void Instructions_RxByteHandler(uint8_t rx_byte)
{
  if (TrackPacket_ParseByte(rx_byte) != 0U)
  {
    return;
  }

  if (rx_byte == '\r')
  {
    return;
  }

  if (rx_byte == '\n')
  {
    if ((g_cmd_len > 0U) && (g_cmd_ready == 0U))
    {
      g_cmd_buffer[g_cmd_len] = '\0';
      g_cmd_ready = 1U;
    }

    g_cmd_len = 0U;
    return;
  }

  if (g_cmd_ready != 0U)
  {
    return;
  }

  if (g_cmd_len < (CMD_BUFFER_SIZE - 1U))
  {
    g_cmd_buffer[g_cmd_len] = (char)rx_byte;
    g_cmd_len++;
  }
  else
  {
    g_cmd_overflow = 1U;
    g_cmd_len = 0U;
  }
}

void Instructions_Process(void)
{
  char cmd_line[CMD_BUFFER_SIZE];
  char *cmd_ptr;
  float pan;
  float tilt;
  int16_t packet_x;
  int16_t packet_y;

  if (g_cmd_overflow != 0U){
    g_cmd_overflow = 0U;
    UART1_SendString("ERR: command too long (max 95 chars).\r\n");
  }

	// Master_Data to Servo_Contral(上位机数据处理完毕，舵机控制入口)
  if (g_packet_ready != 0U){
    __disable_irq();
    packet_x = g_packet_x;
    packet_y = g_packet_y;
    g_packet_ready = 0U;
    __enable_irq();
		
		//UART1_SendFormat("x= %d,y= %d",packet_x,packet_y);
    Servo_Contral((float)packet_x, (float)packet_y);
  }

  if (g_cmd_ready == 0U){
    return;
  }

  __disable_irq();
  memcpy(cmd_line, g_cmd_buffer, CMD_BUFFER_SIZE);
  g_cmd_ready = 0U;
  __enable_irq();

  cmd_ptr = TrimLeft(cmd_line);
  ToUppercase(cmd_ptr);
	
	//UART Test_Part
  if (strcmp(cmd_ptr, "HELP") == 0){
    UART1_SendString("Commands:\r\n");
    UART1_SendString("  HELP\r\n");
    UART1_SendString("  STATUS\r\n");
    UART1_SendString("  CENTER\r\n");
		UART1_SendString("  SERVO STOP\r\n");
    UART1_SendString("  PAN <speed>\r\n");
    UART1_SendString("  TILT <speed>\r\n");
    UART1_SendString("  PT <pan> <tilt>\r\n");
		UART1_SendString("  PT_SPEED <x> <y>\r\n");
    UART1_SendString("  TUNE\r\n");
	    UART1_SendString("  BIN: FF AA XH XL YH YL SUM 55\r\n");
    return;
  }

  if ((strcmp(cmd_ptr, "STATUS") == 0) || (strcmp(cmd_ptr, "SERVO STATUS") == 0)){
    UART1_SendFormat("OK PAN=%.2f TILT=%.2f P_MODE=%d T_MODE=%d CHK_ERR=%u\r\n",
                     Servo_GetPanSpeed(),
                     Servo_GetTiltSpeed(),
                     (int)Servo_GetPanMode(),
	                     (int)Servo_GetTiltMode(),
	                     g_packet_checksum_error);
    return;
  }

  if (strcmp(cmd_ptr, "CENTER") == 0){
    Servo_SetPanTilt(0.0f, 0.0f);
    UART1_SendString("OK CENTER\r\n");
    return;
  }
	
	if (strcmp(cmd_ptr, "TUNE") == 0){
	    UART1_SendFormat("ZONES: FAR=%.0f MID=%.0f NEAR=%.0f\r\n"
	                     "GAINS: KP=%.3f KI=%.3f KD=%.3f\r\n"
	                     "MIN_SPD: PAN=%.3f TILT=%.3f DUTY_MS=%u\r\n"
	                     "FAR_SPD: PAN=%.2f TILT=%.2f MID_MAX:%.2f\r\n"
	                     "DIR: PAN=%.0f TILT=%.0f\r\n",
	                     ZONE_FAR, ZONE_MID, ZONE_NEAR,
	                     PAN_KP, PAN_KI, PAN_KD,
	                     PAN_MIN_EFFECTIVE_SPEED, TILT_MIN_EFFECTIVE_SPEED,
	                     DUTY_CYCLE_PERIOD_MS,
	                     PAN_FAR_SPEED, TILT_FAR_SPEED, PAN_MID_SPEED_MAX,
	                     PAN_DIR, TILT_DIR);
	    return;
	  }

		if (strcmp(cmd_ptr, "SERVO STOP") == 0){
		//PWM停止函数
		UART1_SendString("OK SERVO STOP\r\n");
		return;
	}
	
	if (strcmp(cmd_ptr, "SERVO START") == 0){
		Servo_Init();
		UART1_SendString("SERVO START OK\r\n");
		return;
	}
	
  if (sscanf(cmd_ptr, "PAN %f", &pan) == 1){
    Servo_SetPanSpeed(pan);
    UART1_SendFormat("OK PAN=%.1f\r\n", Servo_GetPanSpeed());
    return;
  }

  if (sscanf(cmd_ptr, "TILT %f", &tilt) == 1){
    Servo_SetTiltSpeed(tilt);
    UART1_SendFormat("OK TILT=%.1f\r\n", Servo_GetTiltSpeed());
    return;
  }

  if (sscanf(cmd_ptr, "PT %f %f", &pan, &tilt) == 2){
    Servo_SetPanTilt(pan, tilt);
    UART1_SendFormat("OK PAN=%.1f TILT=%.1f\r\n", Servo_GetPanSpeed(), Servo_GetTiltSpeed());
    return;
  }
	if (sscanf(cmd_ptr, "PT_SPEED %f %f",&pan, &tilt) == 2){
		Servo_Contral(pan,tilt);
		UART1_SendFormat("OK PAN SPEED=%.1f TILT SPEED=%.1f\r\n", Servo_GetPanSpeed(), Servo_GetTiltSpeed());
		return;
	}

  UART1_SendString("ERR: unknown command. Type HELP.\r\n");
}
