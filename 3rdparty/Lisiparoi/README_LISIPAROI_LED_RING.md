3rdparty/Lisiparoi/README_LISIPAROI_LED_RING.md

# LISIPAROI LED Control Options

## 1. Always On


5V → LISIPAROI 5V
GND → LISIPAROI GND
GPIO pad not connected


---

## 2. On/Off Control Using GPIO


5V → LISIPAROI 5V
GND → LISIPAROI GND

Pi GPIO → LISIPAROI GPIO

10kΩ resistor:
LISIPAROI GPIO → GND


**Notes:**
- The 10kΩ resistor acts as a pull-down.
- Ensures the LED stays OFF during boot or when GPIO is floating.

---

## 3. Brightness Control (PWM via GPIO)


5V → LISIPAROI 5V
GND → LISIPAROI GND

Pi PWM GPIO → LISIPAROI GPIO

10kΩ resistor:
LISIPAROI GPIO → GND


**Notes:**
- Use PWM from the Pi to control brightness.
- This is the preferred method.

---

## ⚠️ Important Notes

- Do **NOT** vary the 5V supply voltage to control brightness.
  - LEDs and onboard driver circuits behave poorly with reduced voltage.
- Keep the 5V supply stable at all times.
- Use GPIO (preferably PWM) for control instead.

---

## 🔧 When is a Transistor Needed?

A transistor (MOSFET) is only required if:

- You want to switch the **5V power line directly**, OR
- The LISIPAROI GPIO input does **not respond well to PWM**

Otherwise, it is **not necessary**.

---

## ✅ Summary

| Mode            | Wiring Complexity | Control Type |
|-----------------|------------------|-------------|
| Always On       | Very simple      | None        |
| On/Off          | Simple           | Digital     |
| Brightness PWM  | Simple           | PWM (best)  |