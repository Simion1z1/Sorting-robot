# Schemă bloc — robot de sortare QR (proiect complet)

Diagramă bloc a întregului sistem: alimentare 12V, cele două ESP32, driverele A4988,
motoarele NEMA17, endstop-urile MIN/MAX, gripper-ul SG90 și camera. Pinii corespund
pin map-ului din `CLAUDE.md` §4.

---

## 1. Schema bloc completă (hardware)

```mermaid
flowchart TB
    %% ===== ALIMENTARE =====
    subgraph PWR["⚡ Alimentare"]
        PSU["PSU 12V<br/>6–8A + siguranta 5A"]
        BUCK33["Buck 12V→3.3V"]
        BUCK5["Buck 12V→5V<br/>+ cap 470µF"]
        PSU --> BUCK33
        PSU --> BUCK5
    end

    %% ===== BRAIN =====
    subgraph BRAIN["🧠 Brain — ESP32 WROOM-32D (masina de stari)"]
        B[ESP32 WROOM-32D]
    end

    %% ===== DRIVERE + MOTOARE =====
    subgraph DRV["⚙️ Drivere + motoare"]
        DX["A4988 X<br/>+ heatsink + cap 100µF"]
        DY["A4988 Y<br/>+ heatsink + cap 100µF"]
        DZ["A4988 Z<br/>+ heatsink + cap 100µF"]
        MX["NEMA17 X<br/>(actuator surub)"]
        MY["NEMA17 Y<br/>(actuator surub)"]
        MZ["NEMA17 Z<br/>(actuator surub)"]
        DX --> MX
        DY --> MY
        DZ --> MZ
    end

    %% ===== ENDSTOP-URI =====
    subgraph ES["🔘 Endstop-uri SS-5GL (NC, INPUT_PULLUP)"]
        EXMIN["X MIN / home"]
        EYMIN["Y MIN / home"]
        EZMIN["Z MIN / home"]
        EXMAX["X MAX"]
        EYMAX["Y MAX"]
        EZMAX["Z MAX"]
    end

    %% ===== CARUCIOR Z (mobil) =====
    subgraph ZCAR["🎥 Carucior Z (mobil)"]
        CAM["ESP32-CAM + OV2640<br/>decode QR (quirc)"]
        SERVO["SG90 — gripper paralel"]
        LED["LED iluminare (GPIO4, optional)"]
        CAM -.-> LED
    end

    %% ===== ALIMENTARE -> CONSUMATORI =====
    BUCK33 -->|3.3V| B
    BUCK33 -->|3.3V VDD logic| DX & DY & DZ
    BUCK5  -->|5V| CAM
    PSU    -->|12V VMOT| DX & DY & DZ
    BUCK33 -->|3.3V / 5V| SERVO

    %% ===== SEMNALE BRAIN -> DRIVERE =====
    B -->|"STEP 25 / DIR 26"| DX
    B -->|"STEP 32 / DIR 33"| DY
    B -->|"STEP 27 / DIR 14"| DZ
    B -->|"ENABLE 13 (shared, active LOW)"| DX & DY & DZ

    %% ===== ENDSTOP-URI -> BRAIN =====
    EXMIN -->|GPIO21| B
    EYMIN -->|GPIO22| B
    EZMIN -->|GPIO23| B
    EXMAX -->|GPIO4| B
    EYMAX -->|GPIO18| B
    EZMAX -->|GPIO17| B

    %% ===== BRAIN <-> CARUCIOR =====
    B -->|"PWM LEDC GPIO19, 50Hz"| SERVO
    B -->|"SCAN trigger GPIO5 → CAM GPIO14"| CAM
    CAM -->|"QR:xx UART GPIO13 → brain GPIO16"| B

    %% ===== GND COMUN =====
    GND{{"⏚ GND COMUN — obligatoriu pentru toate"}}
    PSU --- GND
    BUCK33 --- GND
    BUCK5 --- GND
    B --- GND
    CAM --- GND
```

---

## 2. Tabel sinteză conexiuni (din §4)

| Bloc | Semnal | Pin brain | Pin destinatie | Tip |
|---|---|---|---|---|
| Motor X | STEP / DIR | 25 / 26 | A4988 X | iesire |
| Motor Y | STEP / DIR | 32 / 33 | A4988 Y | iesire |
| Motor Z | STEP / DIR | 27 / 14 | A4988 Z | iesire |
| Drivere | ENABLE (shared) | 13 | A4988 ×3 | iesire, active LOW |
| Endstop X/Y/Z MIN | home | 21 / 22 / 23 | switch NC → GND | `INPUT_PULLUP` |
| Endstop X/Y/Z MAX | limita | 4 / 18 / 17 | switch NC → GND | `INPUT_PULLUP` |
| Gripper | PWM | 19 | SG90 | LEDC 50Hz |
| CAM trigger | UART2 TX | 5 | CAM GPIO14 | `SCAN\n` |
| CAM date | UART2 RX | 16 | CAM GPIO13 | `QR:...\n` |
| LED status | optional | 2 | LED | iesire |

---

## 3. Magistrale de alimentare

```
PSU 12V ─┬─ VMOT A4988 ×3      (+ cap 100µF / driver)
         ├─ Buck 12V→3.3V ──── 3V3 brain + VDD logic drivere
         └─ Buck 12V→5V ────── 5V ESP32-CAM (+ cap 470µF)

⏚ GND COMUN între: PSU, ambele buck-uri, ambele ESP32, drivere, servo.
```

### Reguli cheie reflectate în schemă
- **VMOT 12V** și **logica 3.3V** sunt domenii separate, unite doar prin **GND comun**.
- **Cap 100µF** pe fiecare VMOT (protectie spike) + **470µF** pe CAM (anti-brownout).
- **ESP32-CAM pe 5V** prin LDO-ul de bord (mai stabil la inrush decat 3.3V direct).
- **Heatsink obligatoriu** pe fiecare A4988 la curentul folosit (~1.1A, Vref ~0.8–0.96V).
- **MS1/MS2/MS3** se seteaza din jumperi pe placa A4988 (nu pe GPIO).
