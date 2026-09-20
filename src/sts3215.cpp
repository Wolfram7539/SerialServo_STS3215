#include "sts3215.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_log.h"

namespace SerialServo
{
  STS3215::STS3215(HardwareSerial *serial, uint8_t rx_pin, uint8_t tx_pin,
                   const uint8_t ids[], size_t servo_count)
      : _rx_pin(rx_pin), _tx_pin(tx_pin)
  {
    _serial = serial;
    _serial->begin(STS_SERIAL_BAUDRATE, STS_SERIAL_MODE, rx_pin, tx_pin);
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < STS_MAX_SERVO_COUNT; i++)
    {
      _servos[i] = nullptr;
      _sts_id2index[i] = 0xFF;
    }
    if (servo_count > STS_MAX_SERVO_COUNT)
      servo_count = STS_MAX_SERVO_COUNT;
    for (size_t i = 0; i < servo_count; i++)
    {
      uint8_t id = ids[i];
      if (id == 0 || _sts_id2index[id] != 0xFF)
        continue;
      _servos[i] = new Servo_info();
      _servos[i]->id = id;
      _servos[i]->position = 0;
      _servos[i]->velocity = 0;
      _servos[i]->mode = 0;
      _sts_id2index[id] = i;
      _servo_count++;
    }
  }

  bool STS3215::isValidId(uint8_t id) const
  {
    return _sts_id2index[id] != 0xFF &&
           _servos[_sts_id2index[id]] != nullptr;
  }

  STS3215::~STS3215()
  {
    vSemaphoreDelete(_mutex);
    for (int i = 0; i < STS_MAX_SERVO_COUNT; i++)
    {
      if (_servos[i] != nullptr)
      {
        delete _servos[i];
        _servos[i] = nullptr;
      }
    }
  }

  void STS3215::Serialbegin()
  {
    _serial->begin(STS_SERIAL_BAUDRATE, STS_SERIAL_MODE, _rx_pin, _tx_pin);
  }

  int STS3215::getPosition(uint8_t id)
  {
    if (!isValidId(id))
      return 0;
    return _servos[_sts_id2index[id]]->position;
  }

  int STS3215::getVelocity(uint8_t id)
  {
    if (!isValidId(id))
      return 0;
    return _servos[_sts_id2index[id]]->velocity;
  }

  void STS3215::sts_writeByteCmd(uint8_t id, uint8_t index, uint8_t data)
  {
    BaseType_t result;
    uint8_t message[8];                     // コマンドパケットを作成
    message[0] = 0xFF;                      // ヘッダ
    message[1] = 0xFF;                      // ヘッダ
    message[2] = id;                        // サーボID
    message[3] = 4;                         // パケットデータ長
    message[4] = 3;                         // コマンド（3は書き込み命令）
    message[5] = index;                     // レジスタ先頭番号
    message[6] = data;                      // 書き込みデータ
    message[7] = sts_calcCkSum(message, 8); // チェックサム
    sts_sendMsgs(message, 8);               // データを送信
  }

  void STS3215::sts_readBytesCmd(uint8_t id, uint8_t index, uint8_t len)
  {
    BaseType_t result;
    uint8_t message[8];                     // コマンドパケットを作成
    message[0] = 0xFF;                      // ヘッダ
    message[1] = 0xFF;                      // ヘッダ
    message[2] = id;                        // サーボID
    message[3] = 4;                         // パケットデータ長
    message[4] = 2;                         // コマンド
    message[5] = index;                     // レジスタ先頭番号
    message[6] = len;                       // 読み込みバイト数
    message[7] = sts_calcCkSum(message, 8); // チェックサム
    sts_sendMsgs(message, 8);               // データを送信
  }

  uint8_t STS3215::sts_calcCkSum(uint8_t arr[], int len)
  {
    int checksum = 0;
    for (int i = 2; i < len - 1; i++)
    {
      checksum += arr[i];
    }
    return ~((uint8_t)(checksum & 0xFF)); // チェックサム
  }

  void STS3215::sts_sendMsgs(uint8_t arr[], int len)
  {
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE)
    {
      for (int i = 0; i < len; i++)
      { // コマンドパケットを送信
        _serial->write(arr[i]);
      }
      xSemaphoreGive(_mutex);
    }
  }

  void STS3215::setID(uint8_t old_id, uint8_t new_id)
  {
    if (!isValidId(old_id) || new_id == 0 || isValidId(new_id))
      return;
    sts_writeByteCmd(old_id, 55, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    sts_writeByteCmd(old_id, 5, new_id); // IDレジスタに新しいIDを書き込む
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    sts_writeByteCmd(old_id, 55, 1);     // 書き込み完了後、通常モードに戻す
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    // 内部データ構造を更新
    // _servos[_sts_id2index[new_id]] = _servos[_sts_id2index[old_id]];
    // _servos[_sts_id2index[old_id]] = nullptr;
    // _servos[_sts_id2index[new_id]]->id = new_id;
    // _sts_id2index[new_id] = _sts_id2index[old_id];
    // _sts_id2index[old_id] = 0xFF; // old_idは無効にする
  }

  void STS3215::setMode(uint8_t id, uint8_t mode)
  {
    if (!isValidId(id) || mode > 2)
      return;
    sts_writeByteCmd(id, 55, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    sts_writeByteCmd(id, 0x08, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    sts_writeByteCmd(id, 0x21, mode);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    sts_writeByteCmd(id, 0x09, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    sts_writeByteCmd(id, 0x0A, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    sts_writeByteCmd(id, 0x0B, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    sts_writeByteCmd(id, 0x0C, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    sts_writeByteCmd(id, 55, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS); // 書き込み後、少し待つ
    _servos[_sts_id2index[id]]->mode = mode;
  }

  void STS3215::writeSpeedM1(uint8_t id, int dir, int speed)
  {
    speed = constrain(speed, 0, 0x7FFF);
    int send_data = ((dir & 1) << 15) + speed;
    byte message[9];
    message[0] = 0xFF; // ヘッダ
    message[1] = 0xFF; // ヘッダ
    message[2] = id;   // サーボID
    message[3] = 5;    // パケットデータ長
    message[4] = 3;    // コマンド（3は書き込み命令）
    message[5] = 0x2E; // レジスタ先頭番号
    message[6] = (send_data) & 0xFF;
    message[7] = (send_data >> 8) & 0xFF;
    message[8] = sts_calcCkSum(message, 9); // チェックサム
    sts_sendMsgs(message, 9);               // データを送信
  }

  void STS3215::writeSpeedM2(uint8_t id, int dir, int speed)
  {
    speed = constrain(speed, 0, 0x03FF);
    int send_data = ((dir & 1) << 10) + speed;
    byte message[9];
    message[0] = 0xFF; // ヘッダ
    message[1] = 0xFF; // ヘッダ
    message[2] = id;   // サーボID
    message[3] = 5;    // パケットデータ長
    message[4] = 3;    // コマンド（3は書き込み命令）
    message[5] = 0x2C; // レジスタ先頭番号
    message[6] = (send_data) & 0xFF;
    message[7] = (send_data >> 8) & 0xFF;
    message[8] = sts_calcCkSum(message, 9); // チェックサム
    sts_sendMsgs(message, 9);               // データを送信
  }

  void STS3215::moveToPosition(uint8_t id, int position)
  {
    byte message[9];
    message[0] = 0xFF; // ヘッダ
    message[1] = 0xFF; // ヘッダ
    message[2] = id;   // サーボID
    message[3] = 5;    // パケットデータ長
    message[4] = 3;    // コマンド（3は書き込み命令）
    message[5] = 42;   // レジスタ先頭番号
    message[6] = (position) & 0xFF;
    message[7] = (position >> 8) & 0xFF;
    message[8] = sts_calcCkSum(message, 9); // チェックサム
    sts_sendMsgs(message, 9);               // データを送信
  }

  bool STS3215::sts_receiveProcess(uint8_t id, uint32_t timeout_us)
  {
    if (!isValidId(id) || _serial == nullptr)
      return false;

    byte message[8] = {0xFF, 0xFF, id, 4, 2, 56, 4, 0};
    message[7] = sts_calcCkSum(message, 8);
    if (xSemaphoreTake(_mutex, portMAX_DELAY) != pdTRUE)
      return false;

    while (_serial->available())
      _serial->read();
    _serial->write(message, sizeof(message));
    _serial->flush();

    constexpr size_t response_size = 10;
    uint8_t response[response_size];
    size_t received = 0;
    uint32_t start_us = micros();
    while ((uint32_t)(micros() - start_us) < timeout_us && received < response_size)
    {
      while (_serial->available() && received < response_size)
        response[received++] = _serial->read();
      yield();
    }
    xSemaphoreGive(_mutex);

    if (received != response_size || response[0] != 0xFF || response[1] != 0xFF ||
        response[2] != id || response[3] != 6 || response[4] != 0)
      return false;

    uint8_t checksum = 0;
    for (size_t i = 2; i < response_size - 1; i++)
      checksum += response[i];
    checksum = ~(checksum & 0xFF);
    if (checksum != response[response_size - 1])
      return false;

    Servo_info *servo = _servos[_sts_id2index[id]];
    servo->position = response[5] | (response[6] << 8);
    int velocity = response[7] | (response[8] << 8);
    servo->velocity = (velocity & 0x8000) ? -(velocity & 0x7FFF) : velocity;
    return true;
  }

} // namespace SerialServo
