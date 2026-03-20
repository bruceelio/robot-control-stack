           +-----------------------------+
           |  12V NiMH Battery (XT30)    |
           +--------------+--------------+
                          |
                       (XT30)
                          |
                    [MAIN FUSE]
                 (e.g., 30–40A)
                          |
                     [E-STOP /]
                     [MAIN SWITCH]
                          |
                          +-----------------------------+
                          |   12V DISTRIBUTION (PDB)    |
                          |  (XT30 PDB or power block)   |
                          +------+-----------+-----------+
                                 |           |
                                 |           |
                            [FUSE 25–30A] [FUSE 25–30A]
                                 |           |
                          +------v---+   +---v------+
                          | RoboClaw |   | RoboClaw |
                          | 2x30 #1  |   | 2x30 #2  |
                          +--+----+--+   +--+----+--+
                             |    |         |    |
                             |    |         |    |
                        Motor A  Motor B  Motor C  Motor D
                        (wheel) (wheel)  (wheel)  (wheel)

                          Encoder A/B     Encoder C/D
                         -> RoboClaw #1  -> RoboClaw #2
                         (quadrature)     (quadrature)

   12V PDB branch #3                         12V PDB branch #4
        |                                        |
    [FUSE 5–7.5A]                            [FUSE 10–15A]
        |                                        |
   +----v-----+                              +---v----------------+
   | 5V Buck  |                              | 6V Buck / UBEC     |
   | 5.1V, 5-8A   |                              | 6.0V, 8–10A        |
   +----+-----+                              +---+----------------+
        |                                        |
   +----v------------------+                 +---v----------------------+
   | Raspberry Pi 4B       |                 | goBILDA Servo PDB (8ch) |
   | (USB-C or 5V GPIO)    |                 +-----------+--------------+
   +----+------------------+                             |
        |                                                |
   USB to Arduino (optional)                         Servos (xN)

B) Fuses, wire gauge, connectors
Fusing (practical starting values)
Main fuse (battery protection)

30A if you want strict protection and expect low traction / no pushing

40A if you want fewer nuisance blows during brief stalls

(Your 4 motors could spike above 30A total in real driving, so 40A main is often the practical choice on a 4-motor strafe bot.)

Per RoboClaw fuses

30A blade fuse (one per controller)

Pi branch

5A fuse

Servo UBEC branch

10A fuse (3 servos total is modest; 10A gives headroom)

Wire gauge

Battery → main fuse → switch → distribution: 14 AWG (or 12 AWG if long run)

Distribution → each RoboClaw: 14–16 AWG

Distribution → 5V buck input: 16–18 AWG

Distribution → 6V UBEC input: 16–18 AWG

Buck → Pi: short + thick (aim equivalent of 18 AWG or better)

UBEC → Servo PDB: 18 AWG is good

Connectors

Keep XT30 for battery and controller feeds (fine at your scale if wiring is sized)

Don’t run the Pi through XT30; use screw terminals or JST/VH depending on your buck

C) Brownout + noise protection (do these, they matter)
1) Keep Pi rail separate (you already planned this)

Dedicated 5V buck for Pi only.

Do not power Pi from servo UBEC or motor controller logic rails.

2) Set Pi rail to 5.1V

Yes — recommended.

Set buck output to 5.1V so the Pi still sees ≥5.0V under load and cable drop.

3) Add capacitors (cheap + effective)

On Pi 5V rail, near Pi input: 1000–2200 µF electrolytic + 0.1 µF ceramic

On servo 6V rail, near Servo PDB: 1000–2200 µF electrolytic

4) Add a TVS diode on the 12V bus (optional but “pro”)

A TVS diode across 12V and GND near the distribution helps clamp spikes.
(If you skip this, it’ll still work; it just improves robustness.)

5) Cable routing rule

Keep motor power wires physically separated from:

Pi power wires

TTL serial wires

encoder wires

If they must cross, cross at 90°.

D) Signal wiring (Pi ↔ RoboClaw TTL, encoders, servos)
1) RoboClaw TTL serial

For reliability with 2 RoboClaws:

Recommended serial approach (no contention)

Use Pi UART for RoboClaw #1

Use a USB-to-TTL adapter for RoboClaw #2 (still TTL into S1, just a separate port)

Why: avoids two devices driving one RX line and keeps two-way comms for both.

Pi UART pins (GPIO header)

TXD0 GPIO14 (Pin 8) → RoboClaw S1 RX

RXD0 GPIO15 (Pin 10) ← RoboClaw S1 TX (recommended for feedback)

GND (Pin 6) ↔ RoboClaw GND

Level shifting: If RoboClaw TX is 5V, protect Pi RX with a divider/level shifter. (If you tell me your exact RoboClaw revision, I’ll state this definitively.)

2) Encoders

Each motor encoder goes to the RoboClaw encoder inputs for that channel:

Encoder A/B + V + GND as per RoboClaw manual

Use twisted pairs for A/GND and B/GND if possible

3) Servos (3 total)

Power: via Servo PDB (6.0V rail)

Control: strongly recommended to avoid jitter:

PCA9685 I²C servo driver from the Pi (hardware-timed pulses)

Pi I²C wiring:

3.3V (Pin 1) → PCA9685 VCC

GND (Pin 6) → PCA9685 GND

SDA GPIO2 (Pin 3) → SDA

SCL GPIO3 (Pin 5) → SCL

Servo signal wires from PCA9685 channels to your servos (signal only; power comes from Servo PDB).

E) Parts list (generic, but complete)
Power + safety

Inline main fuse holder + 40A fuse (and a spare 30A/40A)

E-stop / main switch rated for ≥40A DC

Distribution: XT30 PDB or a high-current distribution block

Per-branch protection

2× inline fuse holders + 30A fuses (RoboClaw feeds)

1× inline fuse holder + 5A fuse (Pi buck feed)

1× inline fuse holder + 10A fuse (servo UBEC feed)

Regulators

5V buck: 5–8A adjustable (set to 5.1V)

6V UBEC: 8–10A (set to 6.0V)

Stabilization

2× electrolytic caps (1000–2200 µF) + a couple 0.1 µF ceramics

Optional: 12V TVS diode across the 12V bus

Control

PCA9685 servo driver board

USB-to-TTL adapter (for RoboClaw #2 serial port)

F) “SR-backwards-compatibility” note

This power system is “FRC-ish capable” because it supports the RoboClaws properly. To be SR-compatible later, you’ll essentially swap:

RoboClaws → SR Motor Board

your distribution → SR Power Board

…and keep Pi + code architecture similar (your IOMap layer).