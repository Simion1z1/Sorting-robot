# Sorting Robot — Ghid de pornire și comenzi

Robot cartezian XYZ care ia cutii din zona **Boxes**, le scanează QR-ul și le sortează pe rafturi
după cod. Acest fișier = pașii de pornire + ce face fiecare comandă.

---

## 1. Cele două variante (alegi una)

| Variantă | Fișiere | Decodare QR | Are nevoie de PC? |
|---|---|---|---|
| **On-device (ESP-NOW)** | `firmware/brain/brain.ino` + `firmware/esp32cam_qr_test/esp32cam_qr_test.ino` | pe ESP32-CAM (quirc) | Nu (standalone) |
| **PC-decode** ⭐ | `firmware/brainPC/brainPC.ino` + `firmware/esp32cam_stream/esp32cam_stream.ino` + `pc/decode_qr.py` | pe PC (OpenCV/pyzbar) | Da |

Varianta **PC-decode** are precizie de citire mult mai mare și modul AUTO (`R1`). Restul ghidului
se axează pe ea; varianta on-device e la fel, doar fără PC (vezi §5).

---

## 2. Înainte de pornire — alimentare (important!)

- **12V** → cele 3 drivere A4988 (stepere).
- **5V dedicat** → servo SG90 (la 4.5V doar ticăie/nu mișcă).
- **ESP32-CAM: alimentată prin USB** — de la **laptop sau power bank** (5V), prin placa
  **ESP32-CAM-MB (HW-381)**. 5V curat, fără buck. *(Power bank: folosește unul fără auto-off,
  altfel se stinge la consum mic.)*
- **GND comun** între brain, drivere, bucks, PSU. **Camera NU** are nevoie de GND comun cu brain-ul:
  link-ul e wireless (WiFi la PC-decode, ESP-NOW la on-device) → camera vrea **doar alimentare USB**,
  niciun fir de date.
- La prima rulare automată: **viteză mică** (`V8`) și mâna pe întrerupător.

---

## 3. Pornire — varianta PC-decode (pas cu pas)

### 3.1 Flash o singură dată
1. **Camera:** încarcă `esp32cam_stream.ino`. Completează întâi:
   ```cpp
   const char *WIFI_SSID = "...";
   const char *WIFI_PASS = "...";
   ```
   La boot, pe serial-ul camerei apare: `Stream: http://192.168.x.x/stream` → **notează IP-ul**.
   Board: *AI Thinker ESP32-CAM*, PSRAM *Enabled*, Partition *Huge APP*.
2. **Brain:** încarcă `brainPC.ino`. La boot scrie `=== BRAIN-PC ... ===`.

### 3.2 Setup PC (o singură dată)
```bash
cd ~/repos/Sorting-robot
sudo apt install -y libzbar0 python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -r pc/requirements.txt
```

### 3.3 De fiecare dată
```bash
cd ~/repos/Sorting-robot
source .venv/bin/activate
python pc/decode_qr.py --url http://192.168.x.x/stream --port /dev/ttyUSB0 --show
```
- `--port`: portul brain-ului (`ls /dev/ttyUSB* /dev/ttyACM*`). Dacă „Permission denied”:
  `sudo usermod -aG dialout $USER` + relogare.
- Comenzile le **tastezi în terminalul scriptului** — le trimite la brain. Răspunsurile brain-ului
  apar ca `[brain] ...`.
- **Nu** ține și Serial Monitor-ul din Arduino deschis pe același port (e ocupat de script).

### 3.4 Operare
```
HA            # homing (Z->X->Y). Așteaptă "[home] ALL done".
V8            # viteză mică prima dată
R1            # AUTO: se duce la SCAN și așteaptă coduri
              #   -> pui o cutie în Boxes -> o duce la celula codului -> revine la SCAN
R0  (sau S)   # oprește modul auto
```

---

## 4. Comenzi (referință completă)

Toate pe serial, 115200, o comandă pe linie.

| Comandă | Ce face |
|---|---|
| `HA` | Homing complet, ordine **Z→X→Y** (anti-coliziune). `HX`/`HY`/`HZ` = o singură axă. |
| `J<axă><mm>` | Jog relativ. Ex: `JX50`, `JZ-10`. Ridică automat Z înainte de mișcări X/Y. |
| `G<axă><mm>` | Mers la poziție absolută (mm). Ex: `GX120`. |
| `C<axă><mm>` | Calibrare `steps_per_mm`: comandă o distanță, apoi tastezi cât a măsurat real. |
| `O` | **Deschide** gripper-ul. |
| `L` | În**chide** gripper-ul (prinde cutia). |
| `E<grade>` | Trimite servo-ul la un unghi (reglaj gripper). Ex: `E47`. |
| `T<slot>` | **Învață** poziția curentă în slot. `TS`=scan, `T11`..`T32`=celule. |
| `M<slot>` | **Mergi** la o poziție învățată. `MS`=scan, `M11`..`M32`=celule. |
| `D` | Afișează toate pozițiile ca **cod C** (de lipit în sketch ca să persiste). |
| `B<cod>` | **Un** ciclu manual: ia din Boxes → pune la celula codului. Ex: `BMS0011`. |
| `Q1` / `Q0` | Auto-sort pe **un** cod QR primit (on/off). Nu se întoarce la SCAN. |
| `R1` / `R0` | **AUTO loop** *(doar brainPC)*: stă la SCAN, sortează fiecare QR, revine la SCAN. |
| `V<mm/s>` | Setează viteza. Ex: `V8`, `V40`. |
| `A<mm/s2>` | Setează accelerația. Ex: `A600`. |
| `S` | **STOP** (oprește mișcarea și modul auto). |
| `P` | Status: poziții, homed (`H`), switch-uri (`m`/`M`), viteză, steps/mm. |
| `?` | Ajutor (lista de comenzi). |

> În varianta **PC-decode**, scriptul mai face: liniile care încep cu `QR:` le trimite la sortare;
> orice altceva tastezi = comandă normală.

---

## 5. Pozițiile și maparea cod → celulă

Rafturi 3 rânduri × 2 coloane (din `Gen Qr-codes/Box Structure.txt`):

| Slot | Celulă | Cod QR |
|---|---|---|
| `M11` | Row1 Col1 | **MS0011** |
| `M12` | Row1 Col2 | **MS0012** |
| `M21` | Row2 Col1 | **CJ0011** |
| `M22` | Row2 Col2 | **CJ0012** |
| `M31` | Row3 Col1 | **EB0011** |
| `M32` | Row3 Col2 | **Boxes** (zona de preluare) |
| `MS`  | — | poziția de scanare (camera deasupra Boxes) |

Maparea cod→celulă e în `CODE_MAP` din sketch. Coduri necunoscute → respinse (nu mișcă).

**Convenție Z:** `Z=0` e sus (home pe MAX). Coborârea spre rafturi = valori **negative** (ex. `GZ-80`).

---

## 6. Pornire — varianta on-device (ESP-NOW, fără PC)

1. Flash `brain.ino` (brain) + `esp32cam_qr_test.ino` (cameră).
2. Alimentează ambele (camera pe 5V dedicat). Fără fir de date — e ESP-NOW (wireless).
3. Pe Serial Monitor-ul brain-ului: `HA`, apoi `Q1`.
4. Camera scanează singură; când vede un cod îl trimite wireless → brain-ul afișează `[CAM] QR:...`
   și, dacă `Q1` e pornit + homed, sortează.
5. *(Nu are modul `R1`.)* Pentru flux controlat, arată QR-ul scurt sau folosește `B<cod>`.

---

## 7. Calibrare & teaching (doar când reconfigurezi)

- **Calibrare distanță:** `HX` → `JX60` (la mijloc) → `CX100` → măsori cu șublerul → tastezi valoarea.
  Apoi scrie `steps_per_mm` rezultat în cod (nu persistă singur).
- **Învățare poziții:** `HA` → jog la fiecare loc (`JX`/`JY`/`JZ-..`) → `T<slot>`. La final `D` →
  lipești blocul în sketch peste `SCAN_POS`/`SHELF` → re-flash.
- **Gripper:** `E<grade>` cu cutia în fălci până strânge ferm fără bâzâit → pune valoarea în
  `grip_close_deg`.

---

## 8. Troubleshooting

| Simptom | Cauză probabilă | Soluție |
|---|---|---|
| Servo doar ticăie / nu mișcă | Alimentare slabă (4.5V) | 5V dedicat, solid |
| Camera nu bootează / se stinge | USB slab sau power bank cu auto-off | USB de laptop bun sau power bank fără auto-off (5V/≥1A) |
| Brain nu primește QR (on-device) | Link/canal ESP-NOW | ambele pe stage corect, re-flash; vezi MAC-uri la boot |
| `[!] ... endstop hit during move` | Mișcare spre un switch | normal la limite; pentru Z, `Z=0` e pe switch → folosim -5mm travel |
| Python: `cv2.error contourArea 0` | bug OpenCV pe cadre degenerate | deja tratat (try/except) — re-rulează |
| Sortează la celula greșită | mapare `CODE_MAP` | vezi §5; verifică `Box Structure.txt` |
| Pierde pași / inexact | viteză/accel prea mari sau Vref mic | scade `V`/`A`, reglează Vref pe A4988 |

---

## 9. Oprire în siguranță

- `S` (sau `R0`) oprește mișcarea și modul auto.
- Pentru oprire fizică: întrerupător pe alimentarea 12V a driverelor (recomandat buton E-stop).
