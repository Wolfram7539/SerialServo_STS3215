#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define STS_TIMEOUT     50  // 応答待ち時間（マイクロ秒）
#define STS_MAX_DATA_LENGTH 256   // 最大データ長（適宜変更してください）
#define STS_SERIAL_BAUDRATE    1000000 // シリアル通信のボーレート
#define STS_SERIAL_MODE SERIAL_8N1 // シリアル通信のモード
#define STS_MAX_SERVO_COUNT 256 // 最大サーボ数
namespace SerialServo{
class STS3215 {
public:
    STS3215(HardwareSerial *serial,uint8_t rx_pin,uint8_t tx_pin,uint8_t ids[]);
    ~STS3215();
    void setID(uint8_t old_id, uint8_t new_id);
    void setMode(uint8_t id, uint8_t mode);
    void writeSpeedM1(uint8_t id, int dir, int speed);
    void writeSpeedM2(uint8_t id, int dir, int speed);
    void moveToPosition(uint8_t id, int position);
    int getPosition(uint8_t id);
    int getVelocity(uint8_t id);
    void sts_receiveProcess(uint8_t id);
private:
    void Serialbegin();
    void sts_readBytesCmd(uint8_t id, uint8_t index, uint8_t len);
    void sts_writeByteCmd(uint8_t id, uint8_t index, uint8_t data);
    void sts_sendMsgs(uint8_t arr[], int len);
    uint8_t sts_calcCkSum(uint8_t arr[], int len);
    
    
    bool sign(float x){return x >= 0 ? 0:1;};
    HardwareSerial* _serial = nullptr;
    SemaphoreHandle_t _mutex;

    uint8_t _buffer[256];
    uint16_t _servo_count = 0;
    uint8_t _rx_pin;
    uint8_t _tx_pin;
    struct Servo_info {
      uint8_t id;
      int position;
      int velocity;
      int mode;
    };
    Servo_info* _servos[STS_MAX_SERVO_COUNT] = {nullptr};
    uint8_t _sts_id2index[STS_MAX_SERVO_COUNT];

};
} // namespace SerialServo
