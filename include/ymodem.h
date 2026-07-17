/*
 * ESP32 YModem
 * Adapted for ESP-IDF v5.5.4
 */

#ifndef __YMODEM_H__
#define __YMODEM_H__

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

//UART 配置
#define EX_UART_NUM         UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_TX_PIN         21          // 可根据硬件修改
#define UART_RX_PIN         20          // 可根据硬件修改
#define UART_BUF_SIZE       1080

// LED 指示传输活动
// 设为 0 则禁用 LED，否则为 LED 连接的 GPIO 编号
#define YMODEM_LED_ACT      8
#define YMODEM_LED_ACT_ON   1   // LED 亮时的电平

// Y-MODEM 协议常量
#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER           (3)
#define PACKET_TRAILER          (2)
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)
#define PACKET_1K_SIZE          (1024)

#define FILE_SIZE_LENGTH        (16)

#define SOH                     (0x01)  /* 128 字节数据包起始 */
#define STX                     (0x02)  /* 1024 字节数据包起始 */
#define EOT                     (0x04)  /* 传输结束 */
#define ACK                     (0x06)  /* 确认 */
#define NAK                     (0x15)  /* 否定确认 */
#define CA                      (0x18)  /* 连续两个 CA 取消传输 */
#define CRC16_REQ               (0x43)  /* 'C' 请求 16 位 CRC */

#define ABORT1                  (0x41)  /* 'A' 用户取消 */
#define ABORT2                  (0x61)  /* 'a' 用户取消 */

#define NAK_TIMEOUT             (1000)
#define MAX_ERRORS              (45)

#define YM_MAX_FILESIZE         (10*1024*1024)  // 最大允许的固件大小

/**
 * @brief 从串口接收文件（YModem receiver）
 * @param ffd      用于保存数据的文件指针（需已打开，可写）
 * @param maxsize  允许的最大文件大小（字节）
 * @param getname  输出缓冲区，用于保存接收到的文件名（可为 NULL）
 * @return         实际接收的文件大小（成功），<0 表示错误码
 */
int Ymodem_Receive(FILE *ffd, unsigned int maxsize, char *getname);

/**
 * @brief 通过串口发送文件（YModem sender）
 * @param sendFileName  要发送的文件名（用于初始包中的文件名）
 * @param sizeFile      文件大小（字节）
 * @param ffd           源文件指针（需已打开，可读）
 * @return              0 成功，非0 错误码
 */
int Ymodem_Transmit(char *sendFileName, unsigned int sizeFile, FILE *ffd);

#ifdef __cplusplus
}
#endif

#endif /* __YMODEM_H__ */
