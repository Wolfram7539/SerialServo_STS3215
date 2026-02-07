#include "sts3215.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_log.h"

namespace SerialServo
{
  STS3215::STS3215(HardwareSerial *serial, uint8_t rx_pin, uint8_t tx_pin, uint8_t ids[]) : _rx_pin(rx_pin), _tx_pin(tx_pin)
  {
    _serial = serial;
    _serial->begin(STS_SERIAL_BAUDRATE, STS_SERIAL_MODE, rx_pin, tx_pin);
    _mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < STS_MAX_SERVO_COUNT; i++)
    {
      _servos[i] = nullptr;
    }
    for (int i = 0; ids[i] != 0; i++)
    {
      _servo_count++;
      uint8_t id = ids[i];
      _servos[i] = new Servo_info();
      _servos[i]->id = id;
      _servos[i]->position = 0;
      _servos[i]->velocity = 0;
      _servos[i]->mode = 0;
      _sts_id2index[id] = i;
    }
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
    if (id >= STS_MAX_SERVO_COUNT || _servos[_sts_id2index[id]] == nullptr)
      return 0;
    return _servos[_sts_id2index[id]]->position;
  }

  int STS3215::getVelocity(uint8_t id)
  {
    if (id >= STS_MAX_SERVO_COUNT || _servos[_sts_id2index[id]] == nullptr)
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
    if (old_id >= STS_MAX_SERVO_COUNT || new_id >= STS_MAX_SERVO_COUNT)
      return;
    if (_servos[_sts_id2index[old_id]] == nullptr)
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
    if (id >= STS_MAX_SERVO_COUNT || _servos[_sts_id2index[id]] == nullptr || mode > 2)
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
    int send_data = (dir << 15) + (speed & 0x7FFF);
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
    int send_data = (dir << 10) + (speed & 0x03FF);
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

  void STS3215::sts_receiveProcess(uint8_t id)
  {

    byte message[8];                        // コマンドパケットを作成
    message[0] = 0xFF;                      // ヘッダ
    message[1] = 0xFF;                      // ヘッダ
    message[2] = id;                        // サーボID
    message[3] = 4;                         // パケットデータ長
    message[4] = 2;                         // コマンド
    message[5] = 56;                        // レジスタ先頭番号
    message[6] = 4;                         // 読み込みバイト数
    message[7] = sts_calcCkSum(message, 8); // チェックサム
    if (_serial == nullptr)
    {
      Serial.println("Serial is nullptr");
      return;
    }
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE)
    {
      for (int i = 0; i < 8; i++)
        _serial->write(message[i]);
      unsigned long startTime = millis();
      while (1)
      {
        if (_serial->available())
        {
          byte d1 = _serial->read();
          byte d2 = _serial->read();
          if (d1 == 0xFF && d2 == 0xFF)
          {
            byte rid = _serial->read();
            byte rlen = _serial->read();
            byte rmode = _serial->read();
            if (rid == id && rlen == 6 && rmode == 0)
            { // IDと長さを確認
              byte calc_checksum = rid + rlen + rmode;
              for (int i = 0; i < rlen - 2; i++)
              {
                _buffer[i] = _serial->read(); // データを読み込み
                calc_checksum += _buffer[i];
              }
              byte checksum = _serial->read();
              calc_checksum = ~(calc_checksum & 0xFF);
              // チェックサムを確認
              // Serial.printf("recv checksum:%x calc:%x\n",checksum,calc_checksum);
              if (checksum == calc_checksum)
              {
                // 正常にデータを受信
                // Serial.printf("Received data for ID %d: ", id);
                int pos = _buffer[0] + (_buffer[1] << 8);
                int vel = _buffer[2] + (_buffer[3] << 8);
                if (vel & 0x8000)
                  vel = -1 * (vel & 0x7FFF);
                if (_servos[_sts_id2index[id]] != nullptr)
                {
                  _servos[_sts_id2index[id]]->position = pos;
                  _servos[_sts_id2index[id]]->velocity = vel;
                }
                else
                {
                  Serial.printf("Error: Servo ID %d not found in internal structure\n", _sts_id2index[id]);
                }
                break;
              }
              else
              {
                Serial.printf("Checksum error for ID %d: received %x, calculated %x\n", id, checksum, calc_checksum);
                break;
              }
            }
          }
        }
        if (millis() - startTime > STS_TIMEOUT)
        {
          // タイムアウト処理
          break;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      xSemaphoreGive(_mutex);
    }
  }

} // namespace SerialServo