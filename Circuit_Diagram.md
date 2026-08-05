# Line Follower Robot — Complete Wiring Reference
**Board:** Arduino Mega 2560 (or Pro Mini variant with same pinout mapping)

---

## 1. Power Distribution (the backbone)

```
3S LiPo (11.1V nominal, ~12.6V full charge), yellow XT60 connector
        │
        ├──> Kill Switch (inline on the POSITIVE wire only)
        │        │
        │        ▼
        │   Switched +11.1V rail
        │        │
        │        ├──────────────────────► DRV8833  VM  (motor power in)
        │        │
        │        └──────────────────────► Buck Converter  IN+
        │
        └──> Battery NEGATIVE ──────────► Common GND rail
                                           (buck IN-, DRV8833 GND, Arduino GND — all tied together)

Buck Converter OUT+ (set to 5V) ────────► Arduino VIN (or 5V pin directly if buck has clean 5V)
Buck Converter OUT- ─────────────────────► Arduino GND
```

**Rules:**
- Kill switch goes only on the battery **positive** lead, before it splits to anything.
- Motors are powered directly from the battery (via DRV8833 VM) — NOT from the buck converter. Motors draw too much current/noise for the logic supply.
- Arduino, OLED, sensor array, and DRV8833 logic pins are powered from the buck converter's regulated 5V.
- **Every GND (battery, buck, DRV8833, Arduino, sensor array, OLED) must tie to one common ground.** This is the #1 cause of "random glitches."

---

## 2. Arduino Mega ↔ DRV8833

| Arduino Mega pin | DRV8833 pin | Purpose |
|---|---|---|
| D9  | AIN1 | Left motor control (PWM) |
| D10 | AIN2 | Left motor control (PWM) |
| D6  | BIN1 | Right motor control (PWM) |
| D7  | BIN2 | Right motor control (PWM) |
| D13 | SLP (nSLEEP/STBY) | HIGH = driver awake |
| 5V  | VCC (logic, if separate from VM) | Logic supply |
| GND | GND | Common ground |

> Pins 9/10/6/7 chosen specifically to avoid Timer0 (shared with pin 13 and `delay()`/`millis()`) and to avoid pins 12/13 which share a timer and PWM-interfere with each other on the Mega. Do **not** substitute pin 2, 3, 4, or 11 for these without checking the Mega timer map again.

## 3. DRV8833 ↔ Motors

| DRV8833 pin | Connects to |
|---|---|
| AO1, AO2 | Left motor (800 RPM) — the two motor leads, either polarity |
| BO1, BO2 | Right motor (800 RPM) — the two motor leads, either polarity |
| VM | Switched battery+ (through kill switch) |
| GND | Common ground |

*If a motor spins the wrong direction once tested, swap that motor's two wires at AO1/AO2 or BO1/BO2 — don't change code for this.*

## 4. Arduino Mega ↔ 8-Channel IR Sensor Array

| Sensor array pin | Arduino Mega pin |
|---|---|
| VCC | 5V (from buck rail) |
| GND | Common ground |
| OUT1 (S0) | A7 |
| OUT2 (S1) | A6 |
| OUT3 (S2) | A5 |
| OUT4 (S3) | A4 |
| OUT5 (S4) | A3 |
| OUT6 (S5) | A2 |
| OUT7 (S6) | A1 |
| OUT8 (S7) | A0 |

*(Matches your code's `sensorPin[8] = {A7,A6,A5,A4,A3,A2,A1,A0}` — order matters for left-to-right sensor alignment.)*

## 5. Arduino Mega ↔ OLED Display (I2C, SSD1306)

| OLED pin | Arduino Mega pin |
|---|---|
| VCC | 5V |
| GND | Common ground |
| SDA | **D20 (SDA)** |
| SCL | **D21 (SCL)** |

> On Mega, I2C is on dedicated pins 20/21 — NOT A4/A5 like an Uno. `Wire.begin()` in your code already targets these automatically; just wire physically to 20/21.

## 6. Arduino Mega ↔ Buttons

| Button | Arduino Mega pin | Other leg |
|---|---|---|
| BTN_START | D10 ⚠️ | GND |
| BTN_STOP | D12 | GND |

⚠️ **Conflict flag:** your code defines `BTN_START` on **D10**, which is the same pin used for `AIN2` (left motor PWM) in this wiring. **These cannot share a pin.** Two options:
- Move `BTN_START` to a free digital pin, e.g. **D3**, and update `#define BTN_START 3` in code, **or**
- Move `AIN2` off D10 to another PWM-capable pin, e.g. D5, and rewire that motor lead accordingly.

Recommended: keep motor pins as documented above (9/10/6/7) and move `BTN_START` to **D3** instead — motors are more sensitive to pin/timer conflicts than a simple digitalRead button.

Both buttons wired as: one leg to the Arduino digital pin, other leg to GND. Code uses `INPUT_PULLUP`, so **no external resistor needed** — pressed = LOW.

---

## 7. Full Point-to-Point Summary Table

| From | To |
|---|---|
| LiPo 3S (+) XT60 | Kill switch in |
| Kill switch out | DRV8833 VM **and** Buck IN+ |
| LiPo 3S (−) XT60 | Common GND bus |
| Buck OUT+ (5V) | Arduino VIN/5V, OLED VCC, Sensor VCC, DRV8833 VCC |
| Buck OUT− | Common GND bus |
| Arduino D9 | DRV8833 AIN1 |
| Arduino D10 | DRV8833 AIN2 |
| Arduino D6 | DRV8833 BIN1 |
| Arduino D7 | DRV8833 BIN2 |
| Arduino D13 | DRV8833 SLP |
| DRV8833 AO1/AO2 | Left motor |
| DRV8833 BO1/BO2 | Right motor |
| Arduino D20 (SDA) | OLED SDA |
| Arduino D21 (SCL) | OLED SCL |
| Arduino A0–A7 | Sensor array OUT8–OUT1 |
| Arduino D3 | BTN_START leg 1 (leg 2 → GND) |
| Arduino D12 | BTN_STOP leg 1 (leg 2 → GND) |

---

## 8. Soldering / assembly notes
- Use thicker gauge wire (18–20 AWG) for the battery → kill switch → DRV8833 VM path; this carries full motor current.
- Use thinner gauge (26–28 AWG) for all logic/signal lines (sensors, I2C, buttons).
- Twist the two motor wires per side together to reduce EMI into the sensor lines.
- Route sensor array wiring away from the motor power wires — keep at least 1–2 cm separation, or shield/twist if they must cross.
- Add a common ground **star point** — one screw terminal or solder junction where LiPo−, buck IN−/OUT−, DRV8833 GND, Arduino GND, sensor GND, and OLED GND all meet — rather than daisy-chaining grounds pin-to-pin.
- Add a flyback/decoupling capacitor (e.g. 100–470 µF electrolytic) across VM/GND right at the DRV8833 if you see resets when motors start — motor inrush current can sag the rail.
