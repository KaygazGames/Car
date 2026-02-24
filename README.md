# RC Car ESP32-S3 Controller (Arduino IDE)

Bu repoda iki ayrı Arduino skeci vardır (araç + el kontrolcüsü):
- `rc_car_controller.ino` → Aracın üstündeki ESP32-S3 firmware
- `esp32_controller.ino` → El kontrolcüsü (ESP32 + joystick + buton)

## Araç Firmware Özellikleri (`rc_car_controller.ino`)
- 2x L298N ile toplam **4 DC motor** sürüşü (sol ön/arka + sağ ön/arka)
- WiFi AP + mDNS (`car.local`)
- Modern web endpointleri:
  - `/mobile`
  - `/desktop` (WASD + H horn)
  - `/debug` (ToF, motor flip, controller mode)
- ToF auto-stop:
  - Engel algılanınca ileri komut bloklanır
  - Geri/sol/sağ devam eder
- Motor hız limiti: `PWM_LIMIT = 200`
- Fail-safe:
  - Ping/kontrol input kesilirse araba durur
  - 500ms sonra soft reset (`ESP.restart()`)
- Pasif buzzer horn
- RGB efekt (WS2812, GPIO48, 3 LED, RBG mantıksal renk sırası)

## El Kontrolcüsü Özellikleri (`esp32_controller.ino`)
- ESP32 kontrolcü, araç AP ağına bağlanır (`Car-Control`)
- Açılışta `POST /api/controller/mode {"enabled":true}` gönderir
- Periyodik olarak `POST /api/controller/input` gönderir:
  - `x`, `y`: analog joystick ADC değerleri
  - `estop`: acil stop butonu (aktif-low)
  - `horn`: joystick SW (aktif-low)
- Joystick center hard-coded:
  - `JOY_CENTER_X = 2048`
  - `JOY_CENTER_Y = 2048`

## Kontrolcü Donanım Pinleri (varsayılan)
`esp32_controller.ino` içindeki varsayılan pinler:
- `JOY_X_PIN = 34`
- `JOY_Y_PIN = 35`
- `JOY_SW_PIN = 25`
- `ESTOP_PIN = 26`

> Kendi kartınıza göre pinleri değiştirebilirsiniz.

## Gerekli Kütüphaneler
Arduino Library Manager:
- `Adafruit VL53L0X`
- `Adafruit NeoPixel`

ESP32 core ile gelen:
- `WiFi.h`
- `WebServer.h`
- `ESPmDNS.h`
- `HTTPClient.h`

## Derleme
Arduino IDE'de:
1. Araç tarafı için `rc_car_controller.ino` yükleyin.
2. El kontrolcüsü için `esp32_controller.ino` yükleyin.


## Notlar
- ESP32 core v3 uyumluluğu için PWM tarafında `ledcAttach(...)` + `ledcWrite(pin, duty)` API yapısı kullanıldı.
- Hem araç hem controller tarafında `Wire.begin()` çağrısı eklidir.
