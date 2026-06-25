# Rulare `decode_qr.py` pe Windows

Scriptul `pc/decode_qr.py` e cross-platform — diferă doar mediul și argumentele față de Linux.
Citește stream-ul MJPEG al ESP32-CAM, decodează QR-ul (pyzbar/OpenCV) și trimite `QR:<cod>`
către brain pe serial USB.

## 1. Portul serial: `COM`, nu `/dev/ttyUSB0`

Pe Windows porturile sunt `COM3`, `COM5` etc. Ca să afli care e brain-ul:

- **Device Manager → Ports (COM & LPT)** → caută:
  - `Silicon Labs CP210x` → ESP32-CAM-MB (CP2102)
  - `USB-SERIAL CH340` → brain-ul ESP32 DevKitC
- sau în PowerShell:
  ```powershell
  [System.IO.Ports.SerialPort]::GetPortNames()
  ```

Brain-ul (cel către care trimitem `QR:...`) e placa DevKitC — pe acest setup chip-ul CH340.

## 2. Driver USB-serial

Pe Linux portul a fost recunoscut automat. Pe Windows, dacă portul nu apare, instalează driverul chip-ului:

- **CP2102** (Silicon Labs) → driver CP210x VCP
- **CH340** (brain) → driver CH340

## 3. Instalare dependențe

Pe Windows **nu** ai nevoie de `libzbar0` separat — wheel-ul `pyzbar` aduce DLL-ul zbar inclus:

```powershell
pip install opencv-python pyzbar pyserial
```

> Dacă pyzbar dă `FileNotFoundError: [WinError 126]`, instalează **Visual C++ Redistributable 2013**
> (zbar DLL depinde de el).

## 4. Comanda de rulare

```powershell
python decode_qr.py --url http://192.168.0.173/stream --port COM5
```

Cu fereastră video:

```powershell
python decode_qr.py --url http://192.168.0.173/stream --port COM5 --show
```

Înlocuiește `COM5` cu portul real al brain-ului. URL-ul camerei rămâne identic cu cel de pe Linux.

## 5. Mediu izolat (venv) — opțional

```powershell
python -m venv .venv
.venv\Scripts\activate          # pe Linux era: source .venv/bin/activate
pip install opencv-python pyzbar pyserial
python decode_qr.py --url http://192.168.0.173/stream --port COM5
```

## Diferențe față de Linux — pe scurt

| Aspect            | Linux                          | Windows                         |
|-------------------|--------------------------------|---------------------------------|
| Port serial       | `--port /dev/ttyUSB0`          | `--port COM5`                   |
| Listare porturi   | `ls /dev/ttyUSB*`              | `[System.IO.Ports.SerialPort]::GetPortNames()` |
| zbar              | `sudo apt install libzbar0`    | inclus în wheel-ul `pyzbar`     |
| Activare venv     | `source .venv/bin/activate`    | `.venv\Scripts\activate`        |
| Driver USB-serial | auto (kernel)                  | CP210x / CH340 manual           |

Codul Python, protocolul serial (115200 8N1, `QR:...\n`) și logica de sortare sunt identice.
