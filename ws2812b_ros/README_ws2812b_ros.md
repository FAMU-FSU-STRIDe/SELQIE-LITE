# 🟢 WS2812B ROS 2 Driver
*A lightweight SPI-based ROS 2 node for controlling WS2812B (NeoPixel) LEDs on NVIDIA Jetson boards.*

![demo](https://github.com/<your-user>/ws2812b_ros/assets/demo.gif)

---

## 📖 Overview
`ws2812b_ros` provides a minimal, timing-safe driver that lets a Jetson Nano, Xavier, or Orin control WS2812B RGB LEDs directly through the SPI bus — no Arduino or microcontroller required.  
The package includes:

- A **driver node** (`led_node`) that listens to `/led_colors` messages.  
- A **test pattern node** (`led_tester`) for color cycling.  
- A **CLI tool** (`ws2812b-set`) to set colors from the terminal.  
- A **demo launch file** to run both nodes together.

---

## ⚙️ Hardware Setup

| Pin | Jetson Pin | Connection | Notes |
|-----|-------------|-------------|-------|
| **MOSI** | Pin 19 | → WS2812B DIN | via 74AHCT125/74HCT14 level shifter |
| **GND** | Pin 6 | → LED GND | common ground |
| **5 V** | Pin 2 or 4 | → LED V<sub>DD</sub> | only if ≤3 LEDs, else external supply |
| — | — | +330 Ω resistor in series on data line |
| — | — | +1000 µF capacitor across 5 V/GND |

> ⚠️ WS2812B requires 5 V logic. Always use a proper 3.3 V→5 V level shifter.

---

## 🧩 Jetson Configuration

Enable SPI:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# → enable SPI1, save, reboot
ls /dev/spidev*
# expect /dev/spidev0.0 or /dev/spidev1.0
```

Grant access:
```bash
sudo usermod -aG spi $USER
newgrp spi
```

---

## 🛠️ Installation

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-spidev
mkdir -p ~/selqie_lite_ws/src
cd ~/selqie_lite_ws/src
git clone https://github.com/<your-user>/ws2812b_ros.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### 1. Run the demo launch
```bash
ros2 launch ws2812b_ros led_demo.launch.py   num_leds:=2 brightness:=0.2 spi_bus:=0 spi_dev:=0 spi_hz:=2400000 pixel_order:=GRB   rate_hz:=5.0 hold_secs:=1.0
```
LEDs will flash red → green → blue → white → off in sequence.

---

### 2. Driver Node
```bash
ros2 run ws2812b_ros led_node --ros-args   -p num_leds:=2 -p brightness:=0.2 -p spi_bus:=0 -p spi_dev:=0   -p spi_hz:=2400000 -p pixel_order:=GRB
```

| Parameter | Type | Default | Description |
|------------|------|----------|-------------|
| `num_leds` | int | 2 | LED count in chain |
| `brightness` | float | 1.0 | Brightness scale (0–1) |
| `spi_bus` / `spi_dev` | int | 0 / 0 | SPI device (e.g., `/dev/spidev0.0`) |
| `spi_hz` | int | 2400000 | SPI clock |
| `pixel_order` | str | `"GRB"` | Channel order (`RGB`, `BGR`, etc.) |

---

### 3. Command-Line Tool 🧠
One-shot publisher for manual control.

```bash
# all off
ws2812b-set --off --num-leds 2

# both red (hex RRGGBB)
ws2812b-set --hex FF0000 --num-leds 2

# LED 1 blue only
ws2812b-set 0 0 255 --num-leds 2 --index 1

# dim white
ws2812b-set --white 64 --num-leds 2
```

Alternatively:
```bash
ros2 run ws2812b_ros ws2812b-set -- --off --num-leds 2
```

---

### 4. Topic Interface
The driver subscribes to:
```
/led_colors   [std_msgs/UInt32MultiArray]
```
Each element in `data[]` is a 24-bit color `0x00RRGGBB`.  
Example:
```bash
ros2 topic pub /led_colors std_msgs/UInt32MultiArray "{data: [16711680, 255]}"
# LED0 red, LED1 blue
```

---

## 🧠 How It Works
Each WS2812B bit is encoded into 3 SPI bits (`1→110`, `0→100`) at ≈2.4 MHz.  
This produces timing-accurate pulses over SPI instead of bit-banging GPIO.

Data flow:
```
ros2 topic  →  led_node (Python)  →  spidev  →  MOSI line  →  WS2812B chain
```

---

## 📂 Package Layout
```
ws2812b_ros/
├── launch/
│   └── led_demo.launch.py
├── ws2812b_ros/
│   ├── led_node.py
│   ├── ws2812b_spi.py
│   ├── led_tester.py
│   └── cli.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 🧪 Troubleshooting

| Symptom | Likely Cause | Fix |
|----------|---------------|-----|
| LEDs dark | Power or GND missing | Verify 5 V and common ground |
| Flicker | No level shifter / poor wiring | Add 74AHCT125 and 330 Ω resistor |
| Wrong colors | Pixel order mismatch | Change `pixel_order` |
| Permission error | User not in `spi` group | `sudo usermod -aG spi $USER` |
| Only first LED works | DIN→DOUT chain break | Check solder or connector |

---

## 🧾 License
MIT License © 2025 Ryan Kaczmarczyk  


---

### 🧰 Maintainer
**Ryan Kaczmarczyk**  
Florida State University – Mechanical Engineering  
