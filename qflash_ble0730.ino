#include <bluefruit.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <SdFat.h>
#include "ff.h"
#include "diskio.h"
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6B);

SPIFlash_Device_t const p25q16h = {
  .total_size = (1UL << 21),
  .start_up_time_us = 10000,
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02,
  .has_sector_protection = 1,
  .supports_fast_read = 1,
  .supports_qspi = 1,
  .supports_qspi_writes = 1,
  .write_status_register_split = 1,
  .single_status_byte = 0,
  .is_fram = 0,
};

SPIClass SPI_2(NRF_SPIM0, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0);
Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, SPI_2);
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
File dataFile;

// BLE
BLEService customService = BLEService(0x180D);
BLECharacteristic ppgChar = BLECharacteristic(0x2A5B);
BLECharacteristic commandChar = BLECharacteristic(0x2A99);

// 핀
const int ppgPin = A0;
bool isLogging = false;
bool isNotifying = false;
unsigned long startLoggingTime = 0;
unsigned long lastSaveMillis = 0;
unsigned long lastSentMillis = 0;
unsigned long saveInterval = 120000;
unsigned long recentDuration = 60000;


static uint8_t sampleCounter = 0;
static uint8_t buffer[20];

bool autoRepeatEnabled = true;  // flutter DATA start or stop button


// 디스크 IO
extern "C" {
  DSTATUS disk_status(BYTE) {
    return 0;
  }
  DSTATUS disk_initialize(BYTE) {
    return 0;
  }
  DRESULT disk_read(BYTE, BYTE *buff, DWORD sector, UINT count) {
    return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }
  DRESULT disk_write(BYTE, const BYTE *buff, DWORD sector, UINT count) {
    return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }
  DRESULT disk_ioctl(BYTE, BYTE cmd, void *buff) {
    switch (cmd) {
      case CTRL_SYNC: flash.syncBlocks(); return RES_OK;
      case GET_SECTOR_COUNT: *((DWORD *)buff) = flash.size() / 512; return RES_OK;
      case GET_SECTOR_SIZE: *((WORD *)buff) = 512; return RES_OK;
      case GET_BLOCK_SIZE: *((DWORD *)buff) = 8; return RES_OK;
      default: return RES_PARERR;
    }
  }
}

void format_fat12(void) {
  uint8_t workbuf[4096];
  FATFS elmchamFatfs;
  FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    Serial.print("포맷 실패! 코드: ");
    Serial.println(r);
    while (1)
      ;
  }
  r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    Serial.println("마운트 실패!");
    while (1)
      ;
  }
  r = f_setlabel("EXT FLASH");
  if (r != FR_OK) {
    Serial.println("라벨 실패!");
    while (1)
      ;
  }
  f_unmount("0:");
  flash.syncBlocks();
  Serial.println("포맷 성공!");
}

void commandCallback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  String cmd = String((char *)data).substring(0, len);
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("start")) {
    int idx1 = cmd.indexOf(":");
    int idx2 = cmd.indexOf(":", idx1 + 1);
    if (idx1 > 0 && idx2 > idx1) {
      saveInterval = cmd.substring(idx1 + 1, idx2).toInt();
      recentDuration = cmd.substring(idx2 + 1).toInt();
      Serial.printf("⏱ 저장 주기: %lu ms, 최근 범위: %lu ms\n", saveInterval, recentDuration);
    }

    isLogging = false;

    Serial.println("📝 저장 설정 완료 (측정은 대기 중)");

  }

  else if (cmd == "measure") {
    // 이 명령이 와야만 실제 측정 시작
    fatfs.remove("ppg.bin");
    dataFile = fatfs.open("ppg.bin", FILE_WRITE);
    if (dataFile) {
      isLogging = true;
      autoRepeatEnabled = true;
      startLoggingTime = millis();
      lastSaveMillis = startLoggingTime;
      Serial.println("✅ 측정 시작");
    }
  }

  else if (cmd == "stop") {
    isLogging = false;
    isNotifying = false;
    autoRepeatEnabled = false;  // ❌ 자동 저장 비활성화
    if (dataFile) dataFile.close();
    Serial.println("🛑 측정 종료");
  }

  else if (cmd == "format") {
    isLogging = false;
    Serial.println("💣 포맷 시작");
    format_fat12();
    fatfs.begin(&flash);
    Serial.println("📂 Flash 포맷 완료");
  }
}

void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start(0);
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  Serial.println("❌ BLE 연결 끊김");

  isLogging = false;
  isNotifying = false;

  if (dataFile && dataFile.isOpen()) {
    dataFile.close();
    Serial.println("🛑 파일 닫힘");
  }

  if (fatfs.exists("ppg.bin")) {
    Serial.println("🔍 ppg.bin 존재함, 삭제 시도");
    if (fatfs.remove("ppg.bin")) {
      Serial.println("🗑️ 연결 끊김으로 파일 삭제됨");
    } else {
      Serial.println("⚠️ 파일 삭제 실패 (연결 끊김)");
    }
  } else {
    Serial.println("ℹ️ 연결 끊김 시 ppg.bin 파일 없음");
  }

  sampleCounter = 0;
  memset(buffer, 0, sizeof(buffer));
}





void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!flash.begin(&p25q16h, true)) {
    Serial.println("❌ flash.begin 실패");
    while (1)
      ;
  }

  if (!fatfs.begin(&flash)) {
    Serial.println("❗ 파일 시스템 마운트 실패. 포맷 시도");
    format_fat12();
    if (!fatfs.begin(&flash)) {
      Serial.println("❌ 마운트 실패");
      while (1)
        ;
    }
  }

  myIMU.begin();

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // 대역폭/메모리 최대로

  Bluefruit.begin();


  // 3) 연결 파라미터 (가능한 짧게 요청)
  Bluefruit.Periph.setConnInterval(6, 12);  // ~7.5~15ms 권장
  Bluefruit.Periph.setConnSlaveLatency(0);
  Bluefruit.Periph.setConnSupervisionTimeout(400);


  Bluefruit.setTxPower(4);
  Bluefruit.setName("PPG_Flash_Device");

  customService.begin();

  ppgChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  ppgChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  ppgChar.setFixedLen(20);  // 2바이트 x 10샘플

  ppgChar.begin();

  commandChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY);
  commandChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  commandChar.setWriteCallback(commandCallback);
  commandChar.begin();

  startAdvertising();
  Serial.println("✅ 초기화 완료");
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
}

void loop() {
  int ppgValue = analogRead(ppgPin);
  unsigned long now = millis();
  static int notifyCount = 0;

  //uint16_t sampleBuffer[10] = {0};
  bool isStill = fabs(myIMU.readFloatGyroX()) < 5 && fabs(myIMU.readFloatGyroY()) < 5 && fabs(myIMU.readFloatGyroZ()) < 5;

  // 데이터 수집
  if (autoRepeatEnabled && Bluefruit.connected() && isLogging && isStill && dataFile && (now - startLoggingTime < recentDuration)) {
    //sampleBuffer[sampleCounter++] = ppgValue;
    buffer[sampleCounter * 2] = ppgValue & 0xFF;
    buffer[sampleCounter * 2 + 1] = (ppgValue >> 8) & 0xFF;
    sampleCounter++;
    Serial.printf("ppgValue: %d, sampleCounter: %d\n", ppgValue, sampleCounter);
    if (sampleCounter >= 10) {
      dataFile.write(buffer, 20);
      flash.syncBlocks();  // 데이터 동기화 추가
      sampleCounter = 0;
    }
  }
  // recentDuration 종료 시 남은 데이터 기록
  if (isLogging && (now - startLoggingTime >= recentDuration) && sampleCounter > 0) {
    dataFile.write(buffer, sampleCounter * 2);  // 실제 채운 바이트 수 기록
    flash.syncBlocks();                         // 데이터 동기화 추가
    Serial.printf("📥 남은 데이터 기록: %d 바이트\n", sampleCounter * 2);
    sampleCounter = 0;
  }

  // recentDuration 종료 후 전송 시작
  if (Bluefruit.connected() && isLogging && (now - startLoggingTime >= recentDuration)) {
    isLogging = false;
    if (dataFile) {
      dataFile.close();
      Serial.println("📥 파일 저장 완료");
    }
    isNotifying = true;
    lastSentMillis = now;
    notifyCount = 0;
    Serial.println("📤 전송 시작");
  }

  // 데이터 전송
  if (isNotifying && ppgChar.notifyEnabled()) {
    File readFile = fatfs.open("ppg.bin", FILE_READ);
    if (!readFile) {
      delay(50);
      readFile = fatfs.open("ppg.bin", FILE_READ);
    }
    if (readFile) {
      while (readFile.available() > 0) {
        int bytesRead = readFile.read(buffer, min(20, readFile.available()));  // 최대 20바이트 읽기
        if (bytesRead > 0) {
          ppgChar.notify(buffer, bytesRead);

          for (int i = 0; i < bytesRead / 2; i++) {
            uint16_t ppg = buffer[i * 2] | (buffer[i * 2 + 1] << 8);
            Serial.print(ppg);
            Serial.print(" ");
          }
          notifyCount++;
          Serial.printf("📤 notify 전송 %d회\n", notifyCount);
          memset(buffer, 0, sizeof(buffer));
          delay(1);  // 너무 빠른 루프 방지
        } else {
          Serial.println("⚠️ 파일 읽기 오류");
          break;
        }
      }
      readFile.close();
      delay(10);
      Serial.println("📤 파일 전송 완료");
      commandChar.notify("done");  // 파일 전송 했으니까 txt파일 만들라는 명령입니다.
      Serial.println("Flutter txt 파일 생성");

    } else {
      Serial.println("⚠️ ppg.bin 파일 열기 실패");
    }

    if (!fatfs.remove("ppg.bin")) {
      delay(20);
      if (!fatfs.remove("ppg.bin")) {
        Serial.println("⚠️ ppg.bin 파일 삭제 재시도 실패");
      }
    } else {
      Serial.println("🗑️ ppg.bin 파일 삭제 완료");
    }

    isNotifying = false;
    Serial.println("✅ isNotifying = false, 전송 완료");
  }

  // 새로운 주기 시작
  if (autoRepeatEnabled && !isLogging && !isNotifying && (now - startLoggingTime >= saveInterval) && startLoggingTime != 0) {
    if (fatfs.exists("ppg.bin")) {
      fatfs.remove("ppg.bin");
    }

    isLogging = true;
    dataFile = fatfs.open("ppg.bin", FILE_WRITE);
    if (dataFile) {
      startLoggingTime = now;
      lastSaveMillis = now;
      Serial.println("♻️ 다음 저장 주기 시작");
    } else {
      Serial.println("⚠️ ppg.bin 파일 열기 실패 (새 주기)");
      isLogging = false;
    }
  }

  delay(4);
}