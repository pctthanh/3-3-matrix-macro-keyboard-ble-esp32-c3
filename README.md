# 3-3-matrix-macro-keyboard-ble-esp32-c3
Macro Keyboard 9 button + Encoder + OLED I2C cho ESP32-C3 Super Mini (3 profile Photoshop &amp; SolidWorks &amp; Capcut)

BLE HID (Keyboard) qua ESP32-BLE-Keyboard (nimble)

(có thể chỉnh sửa chức năng mỗi phím)

3 profile: Photoshop, SolidWorks, Video Edit — chuyển profile bằng cách nhấn encoder (vòng 1→2→3→1)

9 nút macro — mỗi nút gửi phím tắt khác nhau theo profile

Rotary encoder: xoay = dịch trái hoặc phải , nhấn = đổi profile

OLED I²C 128×32 hiển thị:

Profile: <name>

enc:xx Z:<n> 

t: hh:mm:ss (uptime)



Giảm tần số CPU xuống 80 MHz để giảm nhiệt

Encoder xử lý bằng quadrature lookup (ổn định khi quay nhanh)


## ⚙️ Yêu cầu phần cứng


ESP32-C3 Super Mini (hoặc module ESP32-C3 tương tự)

OLED I²C 0.91" hoặc 0.96" (SSD1306, 128×32)

Rotary encoder (EC11 hoặc tương đương, 3 chân: CLK/DT/SW)

9 switch (push buttons)

Dây nối / breadboard / nguồn type c



## 🧰 Thư viện cần cài (Arduino IDE → Library Manager)


NimBLE-Arduino (h2zero)

ESP32-BLE-Keyboard (T-vK) — chọn nhánh nimble nếu có

Adafruit SSD1306

Adafruit GFX

"chương trình đc hỗ trợ với chatgpt ae cần sửa gì thì cứ ném cho chatgpt"
