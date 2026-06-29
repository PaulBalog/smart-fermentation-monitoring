# Smart Fermentation Monitoring

Sistem inteligent pentru **monitorizarea automată a fermentației berii**, construit pe Raspberry Pi, microcontroller STM32 și procesare de imagine cu OpenCV. Aplicația măsoară în timp real grosimea stratului de spumă din bioreactor, colectează parametrii de proces transmiși de bioreactor pe serial (temperatură, pH, pO₂ etc.), controlează iluminarea de captură printr-un releu comandat de STM32 și afișează totul într-o interfață web.

> Proiect de licență — sistem complet hardware + software (firmware, backend, frontend).

---

## Cuprins

- [Cum funcționează](#cum-funcționează)
- [Arhitectură](#arhitectură)
- [Structura proiectului](#structura-proiectului)
- [Componente](#componente)
  - [Firmware (STM32G0B1RE)](#firmware-stm32g0b1re)
  - [Backend (Raspberry Pi / Flask)](#backend-raspberry-pi--flask)
  - [Frontend (interfață web)](#frontend-interfață-web)
- [Hardware necesar](#hardware-necesar)
- [Instalare și rulare](#instalare-și-rulare)
- [Comenzi STM32](#comenzi-stm32)
- [API backend](#api-backend)

---

## Cum funcționează

1. O **cameră** filmează continuu reactorul (sticla de fermentație). Fluxul live este difuzat în interfața web.
2. La fiecare 2 minute, backend-ul **aprinde automat lumina** (printr-un releu comandat de STM32), capturează un cadru luminos și o **stinge înapoi** — ca să nu țină becul aprins inutil.
3. Cadrul este procesat cu OpenCV pentru a determina **grosimea stratului de spumă** (vezi pipeline-ul de mai jos). Valoarea este salvată în `dataset.csv`.
4. În paralel, **bioreactorul** transmite pe portul serial (RS232) un raport cu parametrii de proces. Backend-ul îl parsează și îl salvează în `reactor_dataset.csv`.
5. Interfața web afișează: video live, ultima imagine procesată, graficul evoluției spumei și toți parametrii bioreactorului în timp real.
6. Când nivelul spumei scade sub 40% din maximul atins, sistemul semnalează că **fermentația s-a încheiat**.

### Pipeline-ul de detecție a spumei (OpenCV)

```
ROI (auto-calibrat) → grayscale + CLAHE + blur
   → Sobel + Otsu  ........... găsește nivelul lichidului
   → segmentare pe culoare ... separă crusta de spumă (HSV)
   → învățare crustă persistentă (filtrare temporală)
   → eliminare crustă + păstrare spumă conectată la lichid
   → estimare grosime strat de spumă (px)
```

Semnalul brut este apoi filtrat (mediană mobilă + limitare de salt + netezire) ca să elimine inerția camerei și zgomotul, înainte de a fi desenat pe grafic.

---

## Arhitectură

```
┌──────────────┐   USB-CDC / UART    ┌─────────────────────┐
│   STM32G0B1  │◄───── 115200 ──────►│                     │
│  (releu +    │   LED_ON/OFF/...    │                     │
│   buton)     │                     │   Raspberry Pi      │     HTTP    ┌───────────┐
└──────────────┘                     │   (Flask backend)   │◄──────────►│  Browser  │
                                     │                     │            │ (frontend)│
┌──────────────┐   RS232 / serial    │   - OpenCV          │            └───────────┘
│  Bioreactor  │────── 9600 8N1 ────►│   - cameră          │
│  (proces)    │   raport parametri  │   - control releu   │
└──────────────┘                     └─────────────────────┘
                                            ▲
                                            │ USB
                                       ┌──────────┐
                                       │  Cameră  │
                                       └──────────┘
```

---

## Structura proiectului

```
smart-fermentation-monitoring/
├── README.md
├── backend/
│   └── Backend_py.py          # Server Flask + procesare OpenCV + serial
├── frontend/
│   └── front_end.html         # Interfața web (dashboard)
└── firmware/
    └── stm32g0b1re/           # Proiect STM32CubeIDE
        ├── Core/              # main.c, întreruperi, configurări HAL
        ├── Drivers/           # CMSIS + STM32G0xx HAL Driver
        ├── Licenta_project.ioc        # Configurația CubeMX
        ├── STM32G0B1RETX_FLASH.ld     # Linker script (flash)
        └── STM32G0B1RETX_RAM.ld       # Linker script (RAM)
```

---

## Componente

### Firmware (STM32G0B1RE)

Rulează pe o placă **Nucleo STM32G0B1RE** și are rol de controller pentru iluminarea de captură:

- **Comandă un releu** (PA5) care aprinde/stinge lumina folosită la capturarea imaginilor.
- **Buton on-board** (PC13) pentru comutarea manuală a luminii, cu debounce.
- **Protocol serial pe USART2** (115200 baud, 8N1): primește comenzi text terminate cu `\r`/`\n` și răspunde cu confirmări.
- La pornire trimite `STM32 READY`.

### Backend (Raspberry Pi / Flask)

[`backend/Backend_py.py`](backend/Backend_py.py) — serverul central. Pornește două fire de execuție în fundal:

- **`capture_task`** — programează captura la fiecare 2 minute, gestionează automat lumina (aprinde cu 30 s înainte, stinge la 30 s după), validează luminozitatea cadrului (cu retry) și rulează detecția de spumă.
- **`reactor_serial_task`** — citește continuu portul serial al bioreactorului, auto-detectează portul (`/dev/ttyUSB0`, `ttyUSB1`, `ttyACM1`...), decodează atât flux ASCII brut cât și capturi Hex (Termite), parsează rândurile de raport și salvează cei 16 parametri.

Caracteristici notabile:

- **Auto-calibrare ROI** — găsește singur zona reactorului în primul cadru valid.
- **Control inteligent al luminii** — nu aprinde becul dacă e deja aprins manual.
- **Robustețe serial** — reconectare automată, logare a fluxului brut pentru diagnostic, mai multe rute de diagnoză.
- **Mod debug** — poate genera date simulate de bioreactor (`REACTOR_DEBUG_MODE`) ca să testezi interfața fără hardware.

Parametrii bioreactorului colectați: `temp_c`, `stirr_rpm`, `ph`, `po2_percent`, `acidt_ml`, `baset_ml`, `subst_ml`, `subs_percent`, `o2_t_l`, `o2_en_percent`, `folet_ml`, `weigh_kg`, `ext_1_percent`, `ext_2_percent`, plus două flag-uri.

### Frontend (interfață web)

[`frontend/front_end.html`](frontend/front_end.html) — un dashboard single-page care afișează:

- Stream video live de la cameră (`/video_feed`).
- Graficul evoluției stratului de spumă (`/graph`).
- Parametrii bioreactorului în timp real + istoric (`/reactor_data`, `/reactor_history`).
- Status sistem: temperatură CPU, conexiune STM32 și bioreactor, stare lumini (`/status`).
- Butoane de control pentru lumini și pentru resetarea datelor.

> Notă: backend-ul servește interfața prin `render_template("index.html")`. Pentru rulare, copiază `frontend/front_end.html` în folderul `templates/` al backend-ului sub numele `index.html`.

---

## Hardware necesar

- **Raspberry Pi** (cu Raspberry Pi OS / Linux)
- **Placă Nucleo STM32G0B1RE**
- **Modul releu** + sursă de lumină (LED/bec)
- **Cameră USB** compatibilă V4L2
- **Bioreactor** cu ieșire serială RS232 (+ adaptor USB-RS232 pentru Pi)

---

## Instalare și rulare

### 1. Firmware

Deschide [`firmware/stm32g0b1re/Licenta_project.ioc`](firmware/stm32g0b1re/Licenta_project.ioc) în **STM32CubeIDE**, compilează și încarcă pe placa Nucleo.

### 2. Backend

Pe Raspberry Pi:

```bash
# Dependențe sistem (OpenCV are nevoie de librării native)
sudo apt update
sudo apt install -y python3-pip python3-opencv

# Dependențe Python
pip3 install flask pyserial numpy matplotlib

# Structura de fișiere așteptată de Flask
cd backend
mkdir -p templates static images
cp ../frontend/front_end.html templates/index.html

# Pornire (portul 80 necesită privilegii)
sudo python3 Backend_py.py
```

Apoi deschide în browser `http://<ip-raspberry-pi>/`.

### Configurare

Constantele din partea de sus a [`backend/Backend_py.py`](backend/Backend_py.py) controlează comportamentul — porturi seriale, intervalul de captură, pragurile de detecție a spumei, controlul automat al luminii etc. Ajustează-le în funcție de setup-ul tău.

---

## Comenzi STM32

Trimise de backend pe serial; STM32 răspunde cu confirmări:

| Comandă       | Efect                          | Răspuns               |
|---------------|--------------------------------|-----------------------|
| `LED_ON`      | Aprinde lumina (releu activ)   | `OK LED_ON`           |
| `LED_OFF`     | Stinge lumina                  | `OK LED_OFF`          |
| `LED_TOGGLE`  | Comută starea                  | `OK LED_ON/OFF`       |
| `STATUS`      | Returnează starea curentă      | `STATUS LED_ON/OFF`   |

---

## API backend

| Rută                       | Descriere                                              |
|----------------------------|-------------------------------------------------------|
| `GET /`                    | Interfața web                                         |
| `GET /video_feed`          | Stream MJPEG live de la cameră                        |
| `GET /status`              | Status complet sistem (JSON)                          |
| `GET /graph`               | Graficul evoluției spumei (PNG)                       |
| `GET /comanda/<tip>`       | Comandă lumini (`aprinde`/`stinge`/`toggle`/`status`)|
| `GET /lumini/status`       | Starea curentă a luminilor                            |
| `GET /reactor_data`        | Ultima citire de la bioreactor                        |
| `GET /reactor_history`     | Istoric bioreactor (`?limit=N`)                       |
| `GET /reactor_serial_status` | Diagnostic detaliat al legăturii seriale            |
| `GET /reactor_reconnect`   | Forțează reconectarea serială (`?port=/dev/ttyUSB0`) |
| `GET /reset?confirm=yes`   | Șterge datele de spumă și recalibrează ROI            |
| `GET /reset_reactor?confirm=yes` | Șterge datele bioreactorului                    |
