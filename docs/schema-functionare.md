# Schemă logică de funcționare — robot de sortare QR

Diagrama fluxului operațional: de la pornire → homing → pick → scan QR → decizie → place → repetă.
Sunt implicate două plăci: **brain** (ESP32 WROOM-32D, mașina de stări) și **ESP32-CAM** (nod de viziune).

---

## 1. Fluxul logic complet (ciclu de sortare)

```mermaid
flowchart TD
    START([Pornire / Reset]) --> INIT[Init drivere A4988<br/>ENABLE LOW, LEDC servo, UART2]
    INIT --> HOME{Homing<br/>Z -> X -> Y}
    HOME -->|endstop MIN atins| ORIGIN[Origine setata<br/>X=0 Y=0 Z=0]
    HOME -->|timeout / fir rupt| FAULT[[FAULT: stop drivere]]

    ORIGIN --> GOPICK[GOTO PICK_POS<br/>Z sus inainte de XY]
    GOPICK --> PICK[Z jos -> inchide gripper SG90 -> Z sus]
    PICK --> GOSCAN[GOTO SCAN_POS]

    GOSCAN --> TRIG[/Brain trimite SCAN\\n catre CAM GPIO5/]
    TRIG --> WAITQR{Astept QR<br/>timeout 3s}

    WAITQR -->|QR:XXyyyy| VALID{Format valid?<br/>prefix MS/BV/CJ + cifre}
    WAITQR -->|NOQR / timeout| REJECT[GOTO reject bin<br/>log eroare]

    VALID -->|da| DECIDE[Prefix -> coloana<br/>prima celula libera]
    VALID -->|nu| REJECT

    DECIDE --> PLACE[GOTO SHELF r,c<br/>Z jos -> deschide gripper -> Z sus]
    PLACE --> FILL[colFill c++]
    FILL --> MORE{Mai sunt piese?}

    REJECT --> MORE
    MORE -->|da| GOPICK
    MORE -->|nu| IDLE([IDLE])
    FAULT --> IDLE
```

---

## 2. Schimbul serial brain ↔ ESP32-CAM (faza de scanare)

```mermaid
sequenceDiagram
    participant B as Brain (WROOM-32D)
    participant C as ESP32-CAM (quirc)

    Note over B: Stare AT_SCAN
    B->>C: SCAN\n   (GPIO5 -> GPIO14)
    activate C
    C->>C: capture OV2640 (QVGA/VGA)
    C->>C: decode QR (quirc)
    alt cod gasit
        C-->>B: QR:CJ0012\n   (GPIO13 -> GPIO16)
    else fara cod
        C-->>B: NOQR\n
    end
    deactivate C
    Note over B: parse linie pana la \n<br/>valideaza prefix + cifre
```

---

## 3. Maparea QR → raft (logica de sortare)

```mermaid
flowchart LR
    QR[QR: ddNNNN] --> PFX{Primele 2 litere}
    PFX -->|MS| C0[Coloana 0 - Mures]
    PFX -->|BV| C1[Coloana 1 - Brasov]
    PFX -->|CJ| C2[Coloana 2 - Cluj]
    PFX -->|alt| RJ[Reject bin]

    C0 --> F0[Rand = colFill 0<br/>SHELF 0..2 , 0]
    C1 --> F1[Rand = colFill 1<br/>SHELF 0..2 , 1]
    C2 --> F2[Rand = colFill 2<br/>SHELF 0..2 , 2]
```

---

### Note de siguranță reflectate în logică
- **Z mereu sus** înainte de orice mișcare X/Y (anti-coliziune cu rafturile).
- **Homing** în ordinea Z→X→Y, cu fast approach → back-off → slow re-approach.
- **Endstop NC** + `INPUT_PULLUP`: fir rupt = HIGH = fault (fail-safe).
- **Timeout scanare** (3 s) → piesa merge în reject bin, nu blochează ciclul.
