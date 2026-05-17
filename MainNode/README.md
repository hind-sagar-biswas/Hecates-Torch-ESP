# Hecate (HCTorch)

Hecate's Torch is a small ESP32-based multi-sensor node for evacuation routing and SOS relay using ESP-NOW. It fuses flame, smoke, temperature, radar and ToF sensors to compute local cost and broadcast `NodePacket` updates to neighbors.

Purpose
- **Goal:** Distributed emergency routing with local sensor fusion and simple Bellman-Ford routing to exits.
- **Platform:** ESP32 (PlatformIO / Arduino framework).

Quick start
- **Build:** `pio run -e nodemcu-32s`
- **Flash:** `pio run -e nodemcu-32s -t upload`
- **Monitor:** `pio device monitor -b 115200`

Where to look
- **Entry:** [src/main.cpp](src/main.cpp)
- **Pins / tuning:** [src/config.h](src/config.h)
- **Packets:** [src/comms/packets.h](src/comms/packets.h)
- **ESP-NOW helper:** [src/comms/espnow..cpp](src/comms/espnow..cpp)
- **Routing & state:** [src/routing/node_state.cpp](src/routing/node_state.cpp)
- **Sensors:** [src/sensors](src/sensors) (mq, radar, tof, temperature, flame)
- **Display:** [src/display/oled.cpp](src/display/oled.cpp), [src/display/leds.cpp](src/display/leds.cpp)

Hardware / wiring summary
- **I2C (shared):** SDA=21, SCL=22 — VL53L1X ToF (0x29) + SSD1306 OLED (0x3C)
- **UART2 (radar):** RX=16, TX=17 (LD2410C)
- **OneWire (temp):** pin 4 — DS18B20
- **Analog ADCs:** MQ-2 (pin 34), MQ-135 (pin 35), Flame analog (pin 26)
- **Flame digital:** pin 27 (active-low)
- **LEDs:** GREEN=18, YELLOW=19, RED=23, BLUE=5
- **Buttons:** ALIVE=13, BLOCKED=14, EXIT=12

Note: some demo knobs/pins are intentionally shared in sim setups — see comments in [src/config.h](src/config.h).

Runtime behavior
- Boot sequence initializes I2C, sensors, display, then ESP-NOW and sends a Hello broadcast.
- `mqInit()` performs a blocking ~30s warm-up during `setup()` (intended for demo reliability).
- Main loop reads sensors, updates derived fusion values, computes `local_cost`, updates mood, neighbor staleness and routing, then broadcasts `NodePacket` when due.
- Buttons: Alive toggle (reset mood/state), Blocked toggle (manual obstruction), Exit toggle (mark node as exit and persist to NVS).

Packet formats (wire)
- `HelloPacket` (12 bytes): `[version][type=0][node_id][floor_id][x(float)][y(float)]` — see [src/comms/packets.h](src/comms/packets.h).
- `NodePacket` (15 bytes): `[version][type=1][node_id][floor_id][mood][best_exit_cost(float)][sos_flag][sos_origin_id][timestamp]`.

Known issues & safety notes
- **OLED init guard:** If the OLED is not present `oledInit()` may return early; `oledUpdate()` should guard against uninitialized display (consider `oled_ok` flag). See [src/display/oled.cpp](src/display/oled.cpp).
- **First-loop dt:** `lastTick` is initialized to `0` which can cause an unusually large `dt` on the first loop iteration; consider initializing in `setup()`.
- **Blocking warm-up:** `mqInit()` uses `delay(30000)` which stalls boot for 30s; convert to non-blocking for production.
- **Packet portability:** Packets include `float` values; raw float bytes are not portable across architectures/endianness — if you mix device types, switch to integer-fixed wire formats.
- **Unauthenticated ESP-NOW:** Broadcasts are unauthenticated and can be spoofed or replayed; add application-level authentication if needed.
- **ADC scaling:** ADC raw values assume 0–4095; calibrate MQ sensors for accurate thresholds.

Troubleshooting
- Verify I2C devices: use serial output logs during `setup()` for ToF/OLED detection messages.
- If the node appears offline, check button wiring (pin definitions in [src/config.h](src/config.h)) and internal pull-down support on your ESP32 board.
- Use `pio device monitor -b 115200` to observe boot prints, HELLO broadcasts, and error messages.

Configuration & tuning
- Edit `src/config.h` to set `NODE_ID`, `FLOOR_ID`, `NODE_X`, `NODE_Y`, and to tune thresholds and weights used by the sensor fusion and routing algorithms.

Development notes
- The routing algorithm uses neighbor `best_exit_cost` values and local `local_cost` to compute a next-hop (`direction`) using a Bellman-Ford-like single-step update in `NodeState::updateRouting()`.
- SOS relays are tracked locally per origin id in `NodeState`.
