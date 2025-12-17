# ESP32-S3 Robot + Edge Impulse (masinnägemine)

Robot kasutab ESP32-S3 mikrokontrollerit ja Edge Impulse mudelit mõõdutopsi tuvastamiseks kaamerapildist.
Tööloogika: kaader -> inference -> liikumine; kui tops lähedal, peatub, teeb väikse “nudge”, haarab servo/gripperiga ja tõstab aktuaatoriga.

## Riistvara
- ESP32-S3 (kaameraliidesega)
- Kaamera (RGB565, QVGA 320x240)
- DRV8833 (TT DC mootorid, 2 rattapaari)
- TB6612FNG (lineaarne aktuaator)
- 3x servo (2x MG90S, 1x MG996R)
- Ultraheliandur(id)
- Toide: aku 6V; mootorid eraldi haru MINI360 kaudu ~3V; servod eraldi 5V haru; ühine GND

### Kaamera ühendused (ESP32-S3)
- XCLK -> GPIO15
- SIOD/SDA -> GPIO4
- SIOC/SCL -> GPIO5
- D0 -> GPIO11
- D1 -> GPIO9
- D2 -> GPIO8
- D3 -> GPIO10
- D4 -> GPIO12
- D5 -> GPIO18
- D6 -> GPIO17
- D7 -> GPIO16
- VSYNC -> GPIO6
- HREF -> GPIO7
- PCLK -> GPIO13
- RESET: ei kasutata
- PWDN: ei kasutata

### Mootorid (DRV8833)
- RIGHT AIN1 -> GPIO45
- RIGHT AIN2 -> GPIO46
- LEFT  BIN1 -> GPIO47
- LEFT  BIN2 -> GPIO48

Kiirust PWM-iga ei moduleerita (mootorid töötavad konstantsel kiirusel).

### Lineaarne aktuaator (TB6612FNG)
- AIN1 -> GPIO40
- AIN2 -> GPIO39
VM: 6V akutoide (vastavalt ehitusele)

### Servod
- front MG90S -> GPIO42
- back  MG90S -> GPIO43
- MG996R (gripper) -> GPIO41
Servode toide: eraldi 5V haru.

### Ultraheliandur #1
- TRIG -> GPIO1
- ECHO -> GPIO2
- VCC -> 5V
- GND -> ühine

### Ultraheliandur #2 (valikuline)
- TRIG -> GPIO19 (USB D+)
- ECHO -> GPIO20 (USB D-)

**Märkus:** kui #2 on GPIO19/20 peal, ei saa samal ajal USB kaudu debuggida/jälgida Serialit. Testimiseks on see koodis välja kommenteeritud.

### I2C
- SDA -> GPIO14
- SCL -> GPIO21

## Kuidas ehitada ja laadida (Arduino IDE)
1. Installi ESP32 board package (Espressif)
2. Ava `src/robot_edge_impulse.ino`
3. Veendu, et `edge-impulse/` kaust on projekti sees (või Edge Impulse library on Arduino Libraries all)
4. Vali õige board (ESP32-S3) ja port
5. Upload
<img width="622" height="861" alt="image" src="https://github.com/user-attachments/assets/68ae742d-7973-4ffe-866a-2c86daf47928" />


