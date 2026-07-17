#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "rom/crc.h"
#include "esp_log.h"
#include "ymodem.h"

static const char *TAG = "ymodem";

// LED 辅助函数
static void LED_toggle(void)
{
#if YMODEM_LED_ACT
  static uint8_t led_state = 0;
  led_state = !led_state;
  gpio_set_level(YMODEM_LED_ACT, led_state ? YMODEM_LED_ACT_ON : (1 - YMODEM_LED_ACT_ON));
#endif
}

static void LED_init(void)
{
#if YMODEM_LED_ACT
  gpio_set_direction(YMODEM_LED_ACT, GPIO_MODE_OUTPUT);
  gpio_set_level(YMODEM_LED_ACT, 1 - YMODEM_LED_ACT_ON);
#endif
}

static void LED_deinit(void)
{
#if YMODEM_LED_ACT
  gpio_set_level(YMODEM_LED_ACT, 1 - YMODEM_LED_ACT_ON);
#endif
}

// CRC16 计算
static unsigned short crc16(const unsigned char *buf, unsigned long count)
{
  unsigned short crc = 0x0000;
  while (count--)
  {
    crc ^= (*buf++ << 8);
    for (int i = 0; i < 8; i++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// UART 字节接收，超时单位 ms
static int32_t Receive_Byte(unsigned char *c, uint32_t timeout_ms)
{
  int len = uart_read_bytes(EX_UART_NUM, c, 1, pdMS_TO_TICKS(timeout_ms));
  if (len <= 0)
  {
    return -1;
  }
  return 0;
}

// 清空 UART 缓冲区
static void uart_consume(void)
{
  uint8_t dummy[64];
  while (uart_read_bytes(EX_UART_NUM, dummy, sizeof(dummy), pdMS_TO_TICKS(100)) > 0)
  {
    ;
  }
}

// 发送一个字节
static void Send_Byte(char c)
{
  uart_write_bytes(EX_UART_NUM, &c, 1);
}

// 协议控制字符发送
static void send_CA(void)
{
  Send_Byte(CA);
  Send_Byte(CA);
}
static void send_ACK(void) { Send_Byte(ACK); }
static void send_ACKCRC16(void)
{
  Send_Byte(ACK);
  Send_Byte(CRC16_REQ);
}
static void send_NAK(void) { Send_Byte(NAK); }
static void send_CRC16(void) { Send_Byte(CRC16_REQ); }

//-----------------------------------------------------------------------------
// 接收一个 YModem 数据包
// 返回值:  0 正常返回
//         -1 超时
//         -2 用户取消
// 参数 length 含义:
//         >0 数据长度
//          0 传输结束 (EOT)
//         -1 发送方取消 (两个 CA)
//         -2 CRC 或序号错误
//-----------------------------------------------------------------------------
static int32_t Receive_Packet(uint8_t *data, int *length, uint32_t timeout_ms)
{
  int count, packet_size, i;
  unsigned char ch;
  *length = 0;

  // 接收第一个字节，判断包类型
  if (Receive_Byte(&ch, timeout_ms) < 0)
  {
    return -1;
  }

  switch (ch)
  {
  case SOH:
    packet_size = PACKET_SIZE;
    break;
  case STX:
    packet_size = PACKET_1K_SIZE;
    break;
  case EOT:
    *length = 0;
    return 0;
  case CA:
    // 第二个 CA 确认取消
    if (Receive_Byte(&ch, timeout_ms) < 0)
    {
      return -2;
    }
    if (ch == CA)
    {
      *length = -1;
      return 0;
    }
    return -1;
  case ABORT1:
  case ABORT2:
    return -2;
  default:
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_consume();
    return -1;
  }

  *data = (uint8_t)ch;
  uint8_t *dptr = data + 1;
  count = packet_size + PACKET_OVERHEAD - 1;

  // 接收剩余字节
  for (i = 0; i < count; i++)
  {
    if (Receive_Byte(&ch, timeout_ms) < 0)
    {
      return -1;
    }
    *dptr++ = (uint8_t)ch;
  }

  // 检查序号正确性
  if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff))
  {
    ESP_LOGE(TAG, "Seq mismatch: got %02x, expected %02x",
             data[PACKET_SEQNO_INDEX], (data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff);
    *length = -2;
    return 0;
  }

  // 读取接收到的 CRC（高字节在前）
  uint16_t recv_crc = (data[packet_size + PACKET_HEADER] << 8) |
                      data[packet_size + PACKET_HEADER + 1];

  // 计算数据区（不含 CRC 字段）的 CRC
  uint16_t calc_crc = crc16(&data[PACKET_HEADER], packet_size);

  ESP_LOGI(TAG, "calc_crc=0x%04X, recv_crc=0x%04X", calc_crc, recv_crc);

  if (calc_crc != recv_crc)
  {
    // 打印调试信息：包类型、长度、前16字节数据
    ESP_LOGE(TAG, "CRC error! pkt_type=0x%02X, size=%d", data[0], packet_size);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, &data[PACKET_HEADER], packet_size + PACKET_TRAILER, ESP_LOG_ERROR);
    *length = -2;
    return 0;
  }

  *length = packet_size;
  return 0;
}

// YModem 接收文件
int Ymodem_Receive(FILE *ffd, unsigned int maxsize, char *getname)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  char file_size[FILE_SIZE_LENGTH + 1];
  uint8_t *file_ptr;
  int i;
  unsigned int file_len = 0, write_len, session_done = 0, file_done = 0;
  unsigned int packets_received = 0, errors = 0;
  int packet_length = 0;
  int eof_cnt = 0;
  unsigned int size = 0; // 文件总大小

  LED_init();

  while (!session_done)
  {
    file_done = 0;
    while (!file_done)
    {
      LED_toggle();
      int ret = Receive_Packet(packet_data, &packet_length, NAK_TIMEOUT);
      switch (ret)
      {
      case 0: // 正常接收
        switch (packet_length)
        {
        case -1: // 发送方取消 (CA CA)
          send_ACK();
          size = -1;
          goto exit;
        case -2: // 数据包错误
          errors++;
          if (errors > 5)
          {
            send_CA();
            size = -2;
            goto exit;
          }
          send_NAK();
          break;
        case 0: // EOT
          eof_cnt++;
          if (eof_cnt == 1)
          {
            send_NAK();
          }
          else
          {
            send_ACKCRC16();
          }
          break;
        default: // 正常数据包
          if (eof_cnt > 1)
          {
            send_ACK();
          }
          else if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff))
          {
            // 序号错误
            errors++;
            if (errors > 5)
            {
              send_CA();
              size = -3;
              goto exit;
            }
            send_NAK();
          }
          else
          {
            if (packets_received == 0)
            {
              // 第一个包：文件名包
              if (packet_data[PACKET_HEADER] != 0)
              {
                errors = 0;
                // 提取文件名
                if (getname)
                {
                  for (i = 0, file_ptr = packet_data + PACKET_HEADER;
                       *file_ptr != 0 && i < 64; i++)
                  {
                    *getname++ = *file_ptr++;
                  }
                  *getname = '\0';
                }
                // 提取文件大小
                for (i = 0, file_ptr = packet_data + PACKET_HEADER;
                     *file_ptr != 0 && i < packet_length; file_ptr++)
                {
                  ;
                }
                file_ptr++; // 跳过 '\0'
                for (i = 0; *file_ptr != ' ' && i < FILE_SIZE_LENGTH; i++)
                {
                  file_size[i] = *file_ptr++;
                }
                file_size[i] = '\0';
                if (strlen(file_size) > 0)
                {
                  size = strtol(file_size, NULL, 10);
                }
                else
                {
                  size = 0;
                }

                if (size < 1 || size > maxsize)
                {
                  send_CA();
                  size = (size > maxsize) ? -9 : -4;
                  goto exit;
                }
                file_len = 0;
                send_ACKCRC16();
              }
              else
              {
                // 空文件名包，结束会话
                errors++;
                if (errors > 5)
                {
                  send_CA();
                  size = -5;
                  goto exit;
                }
                send_NAK();
              }
            }
            else
            {
              // 数据包
              if (file_len < size)
              {
                file_len += packet_length;
                if (file_len > size)
                {
                  write_len = packet_length - (file_len - size);
                  file_len = size;
                }
                else
                {
                  write_len = packet_length;
                }
                int written = fwrite(packet_data + PACKET_HEADER, 1, write_len, ffd);
                if (written != write_len)
                {
                  send_CA();
                  size = -6;
                  goto exit;
                }
                LED_toggle();
              }
              errors = 0;
              send_ACK();
            }
            packets_received++;
          }
          break;
        }
        break;
      case -2: // 用户取消
        send_CA();
        size = -7;
        goto exit;
      default: // 超时
        if (eof_cnt > 1)
        {
          file_done = 1;
        }
        else
        {
          errors++;
          if (errors > MAX_ERRORS)
          {
            send_CA();
            size = -8;
            goto exit;
          }
          send_CRC16();
        }
        break;
      } // switch(ret)

      if (file_done)
      {
        session_done = 1;
        break;
      }
    } // while(!file_done)
  }

exit:
  LED_deinit();
  return (int)size;
}

// 以下为 YModem 发送端（还没测试,我前面是用python模拟的）
static void Ymodem_PrepareIntialPacket(uint8_t *data, const char *fileName, uint32_t length)
{
  uint16_t tempCRC;
  memset(data, 0, PACKET_SIZE + PACKET_HEADER);
  data[0] = SOH;
  data[1] = 0x00;
  data[2] = 0xff;

  sprintf((char *)(data + PACKET_HEADER), "%s", fileName);
  sprintf((char *)(data + PACKET_HEADER + strlen((char *)(data + PACKET_HEADER)) + 1), "%lu", length);
  data[PACKET_HEADER + strlen((char *)(data + PACKET_HEADER)) + 1 +
       strlen((char *)(data + PACKET_HEADER + strlen((char *)(data + PACKET_HEADER)) + 1))] = ' ';

  tempCRC = crc16(&data[PACKET_HEADER], PACKET_SIZE);
  data[PACKET_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

static void Ymodem_PrepareLastPacket(uint8_t *data)
{
  uint16_t tempCRC;
  memset(data, 0, PACKET_SIZE + PACKET_HEADER);
  data[0] = SOH;
  data[1] = 0x00;
  data[2] = 0xff;
  tempCRC = crc16(&data[PACKET_HEADER], PACKET_SIZE);
  data[PACKET_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

static void Ymodem_PreparePacket(uint8_t *data, uint8_t pktNo, uint32_t sizeBlk, FILE *ffd)
{
  uint16_t i, size;
  uint16_t tempCRC;

  data[0] = STX;
  data[1] = (pktNo & 0xff);
  data[2] = (~(pktNo & 0xff));

  size = sizeBlk < PACKET_1K_SIZE ? sizeBlk : PACKET_1K_SIZE;
  if (size > 0)
  {
    size = fread(data + PACKET_HEADER, 1, size, ffd);
  }
  if (size < PACKET_1K_SIZE)
  {
    for (i = size + PACKET_HEADER; i < PACKET_1K_SIZE + PACKET_HEADER; i++)
    {
      data[i] = 0x00;
    }
  }
  tempCRC = crc16(&data[PACKET_HEADER], PACKET_1K_SIZE);
  data[PACKET_1K_SIZE + PACKET_HEADER] = tempCRC >> 8;
  data[PACKET_1K_SIZE + PACKET_HEADER + 1] = tempCRC & 0xFF;
}

static uint8_t Ymodem_WaitResponse(uint8_t ackchr, uint8_t tmo)
{
  unsigned char receivedC;
  uint32_t errors = 0;
  do
  {
    if (Receive_Byte(&receivedC, NAK_TIMEOUT) == 0)
    {
      if (receivedC == ackchr)
        return 1;
      else if (receivedC == CA)
      {
        send_CA();
        return 2;
      }
      else if (receivedC == NAK)
        return 3;
      else
        return 4;
    }
    else
    {
      errors++;
    }
  } while (errors < tmo);
  return 0;
}

int Ymodem_Transmit(char *sendFileName, unsigned int sizeFile, FILE *ffd)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint16_t blkNumber;
  unsigned char receivedC;
  int err = 0;
  unsigned int size = sizeFile;

  // 等待接收方发送 'C'
  err = 0;
  do
  {
    Send_Byte(CRC16_REQ);
    LED_toggle();
  } while (Receive_Byte(&receivedC, NAK_TIMEOUT) < 0 && err++ < 45);
  if (err >= 45 || receivedC != CRC16_REQ)
  {
    send_CA();
    return -1;
  }

  // 发送第一个文件名包
  Ymodem_PrepareIntialPacket(packet_data, sendFileName, sizeFile);
  do
  {
    uart_write_bytes(EX_UART_NUM, (char *)packet_data, PACKET_SIZE + PACKET_OVERHEAD);
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 0 || err == 4)
    {
      send_CA();
      return -2;
    }
    else if (err == 2)
      return 98;
    LED_toggle();
  } while (err != 1);

  // 接收方在 ACK 后会发送 'C' 开始数据块传输
  if (Ymodem_WaitResponse(CRC16_REQ, 10) != 1)
  {
    send_CA();
    return -3;
  }

  blkNumber = 0x01;
  while (size > 0)
  {
    Ymodem_PreparePacket(packet_data, blkNumber, size, ffd);
    do
    {
      uart_write_bytes(EX_UART_NUM, (char *)packet_data, PACKET_1K_SIZE + PACKET_OVERHEAD);
      err = Ymodem_WaitResponse(ACK, 10);
      if (err == 1)
      {
        blkNumber++;
        if (size > PACKET_1K_SIZE)
          size -= PACKET_1K_SIZE;
        else
          size = 0;
      }
      else if (err == 0 || err == 4)
      {
        send_CA();
        return -4;
      }
      else if (err == 2)
        return -5;
    } while (err != 1);
    LED_toggle();
  }

  // 发送 EOT
  Send_Byte(EOT);
  do
  {
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 3)
      Send_Byte(EOT);
    else if (err == 0 || err == 4)
    {
      send_CA();
      return -6;
    }
    else if (err == 2)
      return -7;
  } while (err != 1);

  // 接收方请求下一个文件，发送结束包
  if (Ymodem_WaitResponse(CRC16_REQ, 10) != 1)
  {
    send_CA();
    return -8;
  }

  Ymodem_PrepareLastPacket(packet_data);
  do
  {
    uart_write_bytes(EX_UART_NUM, (char *)packet_data, PACKET_SIZE + PACKET_OVERHEAD);
    err = Ymodem_WaitResponse(ACK, 10);
    if (err == 0 || err == 4)
    {
      send_CA();
      return -9;
    }
    else if (err == 2)
      return -10;
  } while (err != 1);

  LED_deinit();
  return 0;
}