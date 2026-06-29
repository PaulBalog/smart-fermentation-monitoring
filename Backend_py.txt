from flask import Flask, render_template, Response, jsonify, send_file, request
import cv2
import serial
from serial.tools import list_ports
import time
import threading
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import re
import logging
from collections import deque
from datetime import datetime

app = Flask(__name__)
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# =========================
# CONFIG
# =========================
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200
REACTOR_SERIAL_PORT = "/dev/ttyUSB0"
REACTOR_SERIAL_PORT_FALLBACKS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM1"]
REACTOR_BAUDRATE = 9600
REACTOR_SERIAL_TIMEOUT = 1
REACTOR_RECONNECT_DELAY_SECONDS = 3
REACTOR_DIAGNOSTIC_LOG_SECONDS = 30
REACTOR_DEBUG_MODE = False
REACTOR_DEBUG_INTERVAL_SECONDS = 5
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# Pentru debug grafic: captura o imagine la 2 minute.
CAPTURE_INTERVAL_SECONDS = 120

# Control automat iluminare pentru captura imaginilor.
# Daca luminile sunt stinse, backend-ul le aprinde cu 30 s inainte de poza
# si le stinge la 30 s dupa poza. Daca sunt deja aprinse manual, le lasa asa.
AUTO_LIGHTS_FOR_CAPTURE = True
LIGHT_PRE_CAPTURE_SECONDS = 30
LIGHT_POST_CAPTURE_SECONDS = 30
CAPTURE_MIN_BRIGHTNESS = 8.0
CAPTURE_RETRY_COUNT = 3
CAPTURE_RETRY_DELAY_SECONDS = 3
CAMERA_WARMUP_FRAMES = 30
CAMERA_WARMUP_FRAME_DELAY_SECONDS = 0.05

DATASET_PATH = "dataset.csv"
REACTOR_DATASET_PATH = "reactor_dataset.csv"
REACTOR_RAW_LOG_PATH = "reactor_serial_raw.bin"
IMAGES_DIR = "images"
GRAPH_PATH = "static/graph.png"

# =========================
# CONFIG PROCESARE IMAGINI
# =========================
ENABLE_CRUST_FILTER = True
CRUST_HISTORY_SIZE = 36
CRUST_MIN_HISTORY = 8
CRUST_PERSISTENCE_THRESHOLD = 0.55
CRUST_COLOR_PERSISTENCE_THRESHOLD = 0.45
CRUST_VERTICAL_BAND = (0.00, 0.70)
CRUST_SAMPLE_RECT_ROI = None

CRUST_HSV_LOW = (8, 35, 45)
CRUST_HSV_HIGH = (42, 255, 230)

FOAM_MIN_VALUE = 75
FOAM_WHITE_MAX_SAT = 115
FOAM_YELLOW_HSV_LOW = (8, 25, 70)
FOAM_YELLOW_HSV_HIGH = (48, 230, 245)
FOAM_TOP_PERCENTILE = 15
FOAM_LIQUID_CONTACT_MARGIN = 28

# Daca vrei recalibrare ROI fara restart, acceseaza /reset?confirm=yes.
roi_coords = None
last_frame = None
camera = None
ser = None
latest_light_state = "necunoscut"
data_lock = threading.Lock()
camera_lock = threading.Lock()
reactor_ser = None
reactor_lock = threading.Lock()
latest_reactor_data = {}
reactor_debug_index = 0
reactor_preferred_port = REACTOR_SERIAL_PORT
reactor_active_port = None
reactor_available_ports = []
reactor_last_error = None
reactor_last_raw_line = None
reactor_last_raw_hex = None
reactor_raw_tail = bytearray()
reactor_last_byte_at = None
reactor_last_valid_at = None
reactor_bytes_received = 0
reactor_lines_received = 0
reactor_valid_rows = 0
reactor_transport_mode = "auto"
reactor_rejected_lines = deque(maxlen=10)
crust_edge_history = deque(maxlen=CRUST_HISTORY_SIZE)
crust_color_history = deque(maxlen=CRUST_HISTORY_SIZE)
crust_mask = None


# =========================
# SERIAL
# =========================
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        time.sleep(2)
        print("[OK] STM conectat")
    except Exception as e:
        print(f"[WARN] STM lipsa: {e}")
        ser = None


init_serial()


STM_COMMANDS = {
    "aprinde": "LED_ON",
    "stinge": "LED_OFF",
    "toggle": "LED_TOGGLE",
    "status": "STATUS",
}


def send_stm_command(command):
    global ser, latest_light_state

    if ser is None or not ser.is_open:
        init_serial()

    if ser is None:
        return False, "STM32 indisponibil"

    try:
        ser.reset_input_buffer()
        ser.write((command + "\n").encode("ascii"))
        ser.flush()

        response = ser.readline().decode("utf-8", errors="replace").strip()
        if not response:
            response = "Comanda trimisa, fara raspuns STM32"

        if "LED_ON" in response:
            latest_light_state = "aprins"
        elif "LED_OFF" in response:
            latest_light_state = "stins"

        return True, response

    except Exception as e:
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        return False, f"Eroare comunicatie STM32: {e}"


def refresh_light_state():
    ok, response = send_stm_command("STATUS")
    if not ok:
        print(f"[WARN] Nu pot citi starea luminilor: {response}")
    return ok, response


def prepare_lights_for_capture():
    """
    Returneaza True daca backend-ul a aprins luminile automat.
    In cazul acesta, dupa captura trebuie stinse din nou.
    """
    if not AUTO_LIGHTS_FOR_CAPTURE:
        return False

    if latest_light_state == "necunoscut":
        refresh_light_state()

    lights_were_already_on = latest_light_state == "aprins"

    ok, response = send_stm_command("LED_ON")
    if not ok:
        print(f"[WARN] Nu pot aprinde luminile pentru captura: {response}")
        return False

    if lights_were_already_on:
        print(f"[LIGHT] Lumini confirmate aprinse pentru captura: {response}")
        return False

    print(f"[LIGHT] Lumini aprinse automat pentru captura: {response}")
    return True


def finish_lights_after_capture(were_auto_enabled):
    if not were_auto_enabled:
        return

    ok, response = send_stm_command("LED_OFF")
    if ok:
        print(f"[LIGHT] Lumini stinse automat dupa captura: {response}")
    else:
        print(f"[WARN] Nu pot stinge luminile dupa captura: {response}")


# =========================
# REACTOR SERIAL DATA
# =========================
REACTOR_COLUMNS = [
    "time",
    "temp_c",
    "stirr_rpm",
    "ph",
    "po2_percent",
    "acidt_ml",
    "baset_ml",
    "subst_ml",
    "subs_percent",
    "o2_t_l",
    "o2_en_percent",
    "folet_ml",
    "weigh_kg",
    "ext_1_percent",
    "ext_2_percent",
    "flag_f",
    "flag_h",
]


def get_reactor_candidate_ports():
    """Returneaza porturile configurate si adaptoarele seriale detectate de Linux."""
    global reactor_available_ports

    ports = []
    details = []

    for port in [reactor_preferred_port, REACTOR_SERIAL_PORT] + REACTOR_SERIAL_PORT_FALLBACKS:
        if port and port != SERIAL_PORT and port not in ports:
            ports.append(port)

    try:
        for port_info in list_ports.comports():
            details.append({
                "device": port_info.device,
                "description": port_info.description,
                "manufacturer": port_info.manufacturer,
                "serial_number": port_info.serial_number,
            })

            if port_info.device != SERIAL_PORT and port_info.device not in ports:
                ports.append(port_info.device)
    except Exception as e:
        print(f"[WARN] Nu pot enumera porturile seriale: {e}")

    reactor_available_ports = details
    return ports


def init_reactor_serial():
    global reactor_ser, reactor_active_port, reactor_last_error
    ports = get_reactor_candidate_ports()

    print(f"[REACTOR] Caut conexiunea seriala. Porturi candidate: {ports}")

    for port in ports:
        try:
            reactor_ser = serial.Serial(
                port=port,
                baudrate=REACTOR_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=REACTOR_SERIAL_TIMEOUT,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            time.sleep(2)
            reactor_active_port = port
            reactor_last_error = None
            print(f"[OK] Reactor serial conectat pe {port} la {REACTOR_BAUDRATE} bps, 8N1")
            print("[REACTOR] Astept raportul transmis de bioreactor...")
            return True
        except Exception as e:
            reactor_last_error = f"{port}: {e}"
            print(f"[WARN] Nu pot deschide reactorul pe {port}: {e}")

    reactor_ser = None
    reactor_active_port = None
    return False


def sanitize_reactor_line(line):
    line = line.replace("\r", "").replace("\x0f", " ")
    line = "".join(ch if ch == "|" or ch == ":" or ch == "." or ch == "-" or ch.isalnum() or ch.isspace() else " " for ch in line)
    return re.sub(r"\s+", " ", line).strip()


def decode_termite_hex_line(line):
    """
    Converteste o linie salvata de Termite in modul Hex View:
    20 31 31 3a ...  11:51 ...
    Returneaza None pentru fluxul ASCII brut transmis direct de reactor.
    """
    match = re.match(
        r"^\s*([0-9A-Fa-f]{2}(?:\s+[0-9A-Fa-f]{2}){0,15})(?:\s|$)",
        line,
    )
    if match is None:
        return None

    try:
        hex_values = re.findall(r"[0-9A-Fa-f]{2}", match.group(1))
        return bytes(int(value, 16) for value in hex_values).decode("latin-1", errors="ignore")
    except Exception:
        return None


def extract_reactor_rows(text_buffer, flush_last_line=False):
    """
    Extrage liniile logice din raport. Accepta CR, LF, CRLF si un rand numeric
    complet care nu a primit inca terminatorul de linie.
    """
    normalized = text_buffer.replace("\r\n", "\n").replace("\r", "\n")
    lines = normalized.split("\n")

    if normalized.endswith("\n") or flush_last_line:
        complete_lines = lines
        remainder = ""
    else:
        complete_lines = lines[:-1]
        remainder = lines[-1]

    # Un rand de date are ora si cel putin 15 separatori. Daca este complet,
    # il procesam chiar daca adaptorul nu a transmis inca CR/LF.
    if remainder.count("|") >= 15 and re.search(r"\d{1,2}:\d{2}\s*\|", remainder):
        complete_lines.append(remainder)
        remainder = ""

    return complete_lines, remainder


def extract_complete_reactor_records(text_buffer):
    """
    Cauta direct in flux randuri numerice complete, chiar daca adaptorul nu
    transmite CR/LF. Ultimul camp al raportului contine cele doua flag-uri F H.
    """
    records = []
    search_position = 0
    consumed_until = 0

    while True:
        start_match = re.search(
            r"(?<!\d)\d{1,2}:\d{2}\s*\|",
            text_buffer[search_position:],
        )
        if start_match is None:
            break

        start = search_position + start_match.start()
        tail = text_buffer[start:]

        for end_match in re.finditer(
            r"\|\s*[-+]?\d+(?:[.,]\d+)?\s+[-+]?\d+(?:[.,]\d+)?(?=\s|$)",
            tail,
        ):
            end = end_match.end()
            candidate = tail[:end]

            if candidate.count("|") < 15:
                continue

            if parse_reactor_line(candidate) is not None:
                records.append(candidate)
                consumed_until = start + end
                search_position = consumed_until
                break
        else:
            # Exista inceputul unui rand, dar acesta nu este complet inca.
            return records, text_buffer[start:]

    if records:
        return records, text_buffer[consumed_until:]

    # Pastreaza doar finalul fluxului pentru a evita cresterea nelimitata.
    return records, text_buffer[-5000:]


def append_reactor_raw_log(raw):
    """Pastreaza fluxul serial brut pentru diagnosticarea adaptorului RS232."""
    global reactor_raw_tail

    reactor_raw_tail.extend(raw)
    if len(reactor_raw_tail) > 512:
        del reactor_raw_tail[:-512]

    try:
        with open(REACTOR_RAW_LOG_PATH, "ab") as raw_file:
            raw_file.write(raw)
    except Exception as e:
        print(f"[WARN] Nu pot salva fluxul serial brut: {e}")


def parse_reactor_line(line):
    """
    Parseaza randuri de forma:
    11:51 | 19.7 | 0 | 4.72 | 11.9 | ... | 0 0
    """
    clean_line = sanitize_reactor_line(line)

    if "|" not in clean_line:
        return None

    # Unele adaptoare pot lasa caractere sau text inaintea orei.
    # Incepem parsarea de la primul camp de forma hh:mm urmat de separator.
    row_start = re.search(r"(?<!\d)\d{1,2}:\d{2}\s*\|", clean_line)
    if row_start is None:
        return None

    clean_line = clean_line[row_start.start():]
    parts = [part.strip() for part in clean_line.split("|")]
    if len(parts) < 10:
        return None

    if not re.match(r"^\d{1,2}:\d{2}$", parts[0]):
        return None

    values = {key: None for key in REACTOR_COLUMNS}
    values["time"] = parts[0]

    for key, raw_value in zip(REACTOR_COLUMNS[1:], parts[1:]):
        numbers = re.findall(r"[-+]?\d+(?:\.\d+)?", raw_value.replace(",", "."))
        if not numbers:
            values[key] = None
            continue

        if key in ("flag_f", "flag_h"):
            values[key] = int(float(numbers[0]))
        else:
            values[key] = float(numbers[0])

    # Ultima coloana poate veni ca "0   0", adica doua flag-uri in aceeasi celula.
    last_numbers = re.findall(r"[-+]?\d+(?:\.\d+)?", parts[-1])
    if len(last_numbers) >= 2:
        values["flag_f"] = int(float(last_numbers[0]))
        values["flag_h"] = int(float(last_numbers[1]))

    values["received_at"] = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return values


def save_reactor_data(data):
    file_exists = os.path.isfile(REACTOR_DATASET_PATH)

    with reactor_lock:
        with open(REACTOR_DATASET_PATH, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["received_at"] + REACTOR_COLUMNS)

            if not file_exists:
                writer.writeheader()

            writer.writerow(data)


def reactor_serial_task():
    global latest_reactor_data, reactor_ser
    global reactor_active_port, reactor_last_error, reactor_last_raw_line
    global reactor_last_raw_hex, reactor_transport_mode
    global reactor_last_byte_at, reactor_last_valid_at
    global reactor_bytes_received, reactor_lines_received, reactor_valid_rows

    if REACTOR_DEBUG_MODE:
        reactor_debug_task()
        return

    transport_buffer = ""
    report_buffer = ""
    last_diagnostic_log = 0.0

    while True:
        try:
            if reactor_ser is None or not reactor_ser.is_open:
                init_reactor_serial()

                if reactor_ser is None:
                    time.sleep(REACTOR_RECONNECT_DELAY_SECONDS)
                    continue

            raw = reactor_ser.read(reactor_ser.in_waiting or 1)
            if not raw:
                now_monotonic = time.monotonic()
                if now_monotonic - last_diagnostic_log >= REACTOR_DIAGNOSTIC_LOG_SECONDS:
                    print(
                        f"[REACTOR WAIT] Port={reactor_active_port or 'niciunul'}, "
                        f"octeti primiti={reactor_bytes_received}, "
                        f"ultima valoare valida={reactor_last_valid_at or 'niciuna'}, "
                        f"ultimii octeti={reactor_last_raw_hex or 'niciunul'}"
                    )
                    last_diagnostic_log = now_monotonic
                time.sleep(0.1)
                continue

            reactor_bytes_received += len(raw)
            reactor_last_byte_at = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            reactor_last_raw_hex = raw.hex(" ")
            append_reactor_raw_log(raw)

            chunk = raw.decode("latin-1", errors="ignore")
            transport_buffer += chunk.replace("\r\n", "\n").replace("\r", "\n")

            now_monotonic = time.monotonic()
            if now_monotonic - last_diagnostic_log >= REACTOR_DIAGNOSTIC_LOG_SECONDS:
                print(
                    f"[REACTOR RX] Port={reactor_active_port}, "
                    f"octeti primiti={reactor_bytes_received}, "
                    f"linii valide={reactor_valid_rows}, "
                    f"ultimii octeti={reactor_last_raw_hex}"
                )
                last_diagnostic_log = now_monotonic

            transport_lines, transport_buffer = extract_reactor_rows(transport_buffer)
            for transport_line in transport_lines:
                decoded_hex_line = decode_termite_hex_line(transport_line)
                if decoded_hex_line is not None:
                    reactor_transport_mode = "termite_hex"
                    report_buffer += decoded_hex_line
                else:
                    if reactor_transport_mode == "auto":
                        reactor_transport_mode = "ascii_raw"
                    report_buffer += transport_line + "\n"

            # Pastreaza bufferele sub control daca primim mult antet sau caractere invalide.
            if len(transport_buffer) > 5000:
                transport_buffer = transport_buffer[-1000:]
            if len(report_buffer) > 10000:
                report_buffer = report_buffer[-3000:]

            report_lines, report_buffer = extract_reactor_rows(report_buffer)
            direct_records, report_buffer = extract_complete_reactor_records(report_buffer)
            report_lines.extend(direct_records)
            for line in report_lines:
                if not line.strip():
                    continue

                reactor_lines_received += 1
                reactor_last_raw_line = sanitize_reactor_line(line)[-500:]
                data = parse_reactor_line(line)

                if data is None:
                    # Afiseaza doar liniile relevante, nu liniile decorative ale raportului.
                    if "|" in line:
                        reactor_rejected_lines.append(reactor_last_raw_line)
                        print(f"[REACTOR SKIP] {reactor_last_raw_line}")
                    continue

                with reactor_lock:
                    latest_reactor_data = data

                save_reactor_data(data)
                reactor_valid_rows += 1
                reactor_last_valid_at = data["received_at"]
                reactor_last_error = None
                print(
                    f"[REACTOR OK] port={reactor_active_port} {data['time']} "
                    f"T={data.get('temp_c')} pH={data.get('ph')} pO2={data.get('po2_percent')}"
                )

        except Exception as e:
            reactor_last_error = str(e)
            print(f"[WARN] Eroare citire reactor serial: {e}")
            try:
                if reactor_ser is not None:
                    reactor_ser.close()
            except Exception:
                pass
            reactor_ser = None
            reactor_active_port = None
            transport_buffer = ""
            report_buffer = ""
            reactor_transport_mode = "auto"
            time.sleep(REACTOR_RECONNECT_DELAY_SECONDS)


def load_reactor_history(limit=200):
    if not os.path.exists(REACTOR_DATASET_PATH):
        return []

    with reactor_lock:
        with open(REACTOR_DATASET_PATH) as f:
            rows = list(csv.DictReader(f))

    return rows[-limit:]


def generate_debug_reactor_data():
    """Genereaza valori simulate pentru testarea frontendului fara reactor conectat."""
    global reactor_debug_index

    reactor_debug_index += 1
    i = reactor_debug_index

    now = datetime.now()
    return {
        "received_at": now.strftime("%Y-%m-%d_%H-%M-%S"),
        "time": now.strftime("%H:%M"),
        "temp_c": round(20.0 + 1.5 * np.sin(i / 10), 1),
        "stirr_rpm": 120 + int(20 * np.sin(i / 8)),
        "ph": round(4.70 - 0.015 * i + 0.04 * np.sin(i / 6), 2),
        "po2_percent": round(max(0, 12.0 - 0.08 * i + 1.5 * np.sin(i / 5)), 1),
        "acidt_ml": round(max(0, i * 0.10), 1),
        "baset_ml": 0.0,
        "subst_ml": 0.0,
        "subs_percent": 0.0,
        "o2_t_l": 0.0,
        "o2_en_percent": 0.0,
        "folet_ml": 0.0,
        "weigh_kg": round(0.00 + 0.01 * np.sin(i / 12), 2),
        "ext_1_percent": 0.0,
        "ext_2_percent": 0.0,
        "flag_f": 0,
        "flag_h": 0,
    }


def reactor_debug_task():
    """Ruleaza cand nu ai reactorul fizic si vrei sa testezi site-ul."""
    global latest_reactor_data

    print("[DEBUG] Reactor simulat activ. Datele sunt generate artificial.")

    while True:
        data = generate_debug_reactor_data()

        with reactor_lock:
            latest_reactor_data = data

        save_reactor_data(data)
        print(
            f"[REACTOR DEBUG] {data['time']} "
            f"T={data['temp_c']} pH={data['ph']} pO2={data['po2_percent']}"
        )

        time.sleep(REACTOR_DEBUG_INTERVAL_SECONDS)


# =========================
# CAMERA
# =========================
def get_camera():
    global camera
    if camera is None or not camera.isOpened():
        camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    return camera


def generate_frames():
    global last_frame
    cam = get_camera()

    while True:
        with camera_lock:
            success, frame = cam.read()
        if not success:
            time.sleep(0.1)
            continue

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        last_frame = frame.copy()

        _, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")

        time.sleep(0.05)


def calculate_frame_brightness(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(np.mean(gray))


def capture_fresh_frame():
    """
    Citeste cadre noi direct din camera, nu din last_frame.
    Ultimele cadre sunt folosite dupa un mic warm-up ca expunerea camerei
    sa aiba timp sa se adapteze la LED-urile aprinse.
    """
    global last_frame

    cam = get_camera()
    frame = None

    with camera_lock:
        for _ in range(CAMERA_WARMUP_FRAMES):
            success, candidate = cam.read()
            if success and candidate is not None:
                frame = candidate
            time.sleep(CAMERA_WARMUP_FRAME_DELAY_SECONDS)

    if frame is None:
        return None

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    last_frame = frame.copy()
    return frame


def capture_valid_frame():
    """
    Returneaza un cadru suficient de luminos pentru procesare.
    Daca imaginea este prea intunecata, reaprinde LED-urile si reincerca.
    """
    for attempt in range(1, CAPTURE_RETRY_COUNT + 1):
        frame = capture_fresh_frame()
        if frame is None:
            print(f"[WARN] Captura esuata la incercarea {attempt}/{CAPTURE_RETRY_COUNT}.")
        else:
            brightness = calculate_frame_brightness(frame)
            if brightness >= CAPTURE_MIN_BRIGHTNESS:
                if attempt > 1:
                    print(f"[OK] Cadru valid dupa retry. Luminozitate medie: {brightness:.2f}")
                return frame, brightness

            print(
                f"[WARN] Cadru prea intunecat la incercarea "
                f"{attempt}/{CAPTURE_RETRY_COUNT}. Luminozitate medie: {brightness:.2f}"
            )

        if attempt < CAPTURE_RETRY_COUNT:
            send_stm_command("LED_ON")
            time.sleep(CAPTURE_RETRY_DELAY_SECONDS)

    return frame, calculate_frame_brightness(frame) if frame is not None else 0.0


# =========================
# IMAGE PROCESSING - FERMENTATIE
# =========================
def auto_calibrate_roi_from_frame(frame):
    """Gaseste automat zona reactorului pe primul cadru valid."""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    col_mean = np.mean(gray, axis=0)
    col_smooth = np.convolve(col_mean, np.ones(50) / 50, mode="same")

    threshold_light = np.mean(col_smooth)
    bright_cols = np.where(col_smooth > threshold_light)[0]

    if len(bright_cols) == 0:
        print("[WARN] Auto-calibrarea ROI a esuat. Folosesc valori default.")
        return int(h * 0.20), int(h * 0.60), int(w * 0.15), int(w * 0.85)

    x_min = bright_cols[0]
    x_max = bright_cols[-1]
    w_glass = x_max - x_min

    x_start = x_min + int(w_glass * 0.10)
    x_end = x_max - int(w_glass * 0.10)
    y_start = int(h * 0.20)
    y_end = int(h * 0.60)

    print(f"[ROI] X({x_start}->{x_end}), Y({y_start}->{y_end})")
    return y_start, y_end, x_start, x_end


def build_rect_mask(shape, rect_fractions):
    mask = np.zeros(shape, dtype=np.uint8)
    if rect_fractions is None:
        mask[:, :] = 255
        return mask

    h, w = shape
    x1_f, y1_f, x2_f, y2_f = rect_fractions
    x1 = max(0, min(w, int(w * x1_f)))
    x2 = max(0, min(w, int(w * x2_f)))
    y1 = max(0, min(h, int(h * y1_f)))
    y2 = max(0, min(h, int(h * y2_f)))

    if x2 > x1 and y2 > y1:
        mask[y1:y2, x1:x2] = 255

    return mask


def preprocess_fermentation_frame(frame, roi):
    y_start, y_end, x_start, x_end = roi
    roi_img = frame[y_start:y_end, x_start:x_end]

    if roi_img.size == 0:
        return None

    gray_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    hsv_roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_roi_clahe = clahe.apply(gray_roi)
    blur_roi = cv2.GaussianBlur(gray_roi_clahe, (5, 5), 0)

    sobel_y = cv2.Sobel(blur_roi, cv2.CV_64F, 0, 1, ksize=3)
    abs_sobel = np.absolute(sobel_y)

    if np.max(abs_sobel) > 0:
        sobel_8u = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    else:
        sobel_8u = np.zeros_like(gray_roi)

    _, edge_bin = cv2.threshold(sobel_8u, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edge_bin = cv2.morphologyEx(edge_bin, cv2.MORPH_CLOSE, kernel_close)
    edge_bin = cv2.morphologyEx(edge_bin, cv2.MORPH_OPEN, kernel_open)

    crust_color = cv2.inRange(
        hsv_roi,
        np.array(CRUST_HSV_LOW, dtype=np.uint8),
        np.array(CRUST_HSV_HIGH, dtype=np.uint8),
    )

    _, s_channel, v_channel = cv2.split(hsv_roi)
    foam_white = ((v_channel >= FOAM_MIN_VALUE) & (s_channel <= FOAM_WHITE_MAX_SAT)).astype(np.uint8) * 255
    foam_yellow = cv2.inRange(
        hsv_roi,
        np.array(FOAM_YELLOW_HSV_LOW, dtype=np.uint8),
        np.array(FOAM_YELLOW_HSV_HIGH, dtype=np.uint8),
    )
    foam_candidate = cv2.bitwise_or(foam_white, foam_yellow)

    kernel_color_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 5))
    kernel_color_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 3))
    crust_color = cv2.morphologyEx(crust_color, cv2.MORPH_OPEN, kernel_color_open)
    foam_candidate = cv2.morphologyEx(foam_candidate, cv2.MORPH_CLOSE, kernel_color_close)
    foam_candidate = cv2.morphologyEx(foam_candidate, cv2.MORPH_OPEN, kernel_color_open)

    return {
        "edge_bin": edge_bin,
        "crust_color": crust_color,
        "foam_candidate": foam_candidate,
    }


def learn_crust_mask_from_history():
    if not ENABLE_CRUST_FILTER:
        return None

    if len(crust_edge_history) < CRUST_MIN_HISTORY or len(crust_color_history) < CRUST_MIN_HISTORY:
        return None

    edge_persistence = np.mean([(mask > 0).astype(np.float32) for mask in crust_edge_history], axis=0)
    color_persistence = np.mean([(mask > 0).astype(np.float32) for mask in crust_color_history], axis=0)

    edge_static = edge_persistence >= CRUST_PERSISTENCE_THRESHOLD
    color_static = color_persistence >= CRUST_COLOR_PERSISTENCE_THRESHOLD
    learned_mask = (color_static | ((color_persistence > 0.25) & edge_static)).astype(np.uint8) * 255

    h, w = learned_mask.shape
    band_mask = np.zeros((h, w), dtype=np.uint8)
    y1 = max(0, min(h, int(h * CRUST_VERTICAL_BAND[0])))
    y2 = max(0, min(h, int(h * CRUST_VERTICAL_BAND[1])))
    band_mask[y1:y2, :] = 255
    learned_mask = cv2.bitwise_and(learned_mask, band_mask)
    learned_mask = cv2.bitwise_and(learned_mask, build_rect_mask((h, w), CRUST_SAMPLE_RECT_ROI))

    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))
    learned_mask = cv2.dilate(learned_mask, kernel_dilate, iterations=1)
    learned_mask = cv2.morphologyEx(learned_mask, cv2.MORPH_OPEN, kernel_open)

    return learned_mask


def update_crust_model(processed):
    global crust_mask

    crust_edge_history.append(processed["edge_bin"])
    crust_color_history.append(processed["crust_color"])
    learned_mask = learn_crust_mask_from_history()

    if learned_mask is not None:
        crust_mask = learned_mask

    return crust_mask


def clean_active_foam_mask(foam_candidate, current_crust_mask):
    if current_crust_mask is None:
        current_crust_mask = np.zeros_like(foam_candidate)

    crust_mask_inv = cv2.bitwise_not(current_crust_mask)
    foam_active = cv2.bitwise_and(foam_candidate, foam_candidate, mask=crust_mask_inv)

    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 7))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 3))
    foam_active = cv2.morphologyEx(foam_active, cv2.MORPH_CLOSE, kernel_close)
    foam_active = cv2.morphologyEx(foam_active, cv2.MORPH_OPEN, kernel_open)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(foam_active, connectivity=8)
    cleaned = np.zeros_like(foam_active)

    if num_labels <= 1:
        return cleaned

    min_area = max(60, int(foam_active.size * 0.002))
    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]
        if area >= min_area:
            cleaned[labels == label] = 255

    return cleaned


def keep_foam_connected_to_liquid(foam_active, y_liquid_roi):
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(foam_active, connectivity=8)
    filtered = np.zeros_like(foam_active)

    if num_labels <= 1:
        return filtered

    h, w = foam_active.shape
    contact_margin = max(FOAM_LIQUID_CONTACT_MARGIN, int(h * 0.10))
    min_area = max(60, int(foam_active.size * 0.002))

    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]
        y = stats[label, cv2.CC_STAT_TOP]
        comp_w = stats[label, cv2.CC_STAT_WIDTH]
        comp_h = stats[label, cv2.CC_STAT_HEIGHT]
        bottom = y + comp_h

        touches_liquid = bottom >= (y_liquid_roi - contact_margin)
        wide_surface_band = comp_w > int(w * 0.18) and bottom >= (y_liquid_roi - 2 * contact_margin)

        if area >= min_area and (touches_liquid or wide_surface_band):
            filtered[labels == label] = 255

    return filtered


def estimate_foam_top_from_mask(foam_active):
    top_points = []

    for x in range(0, foam_active.shape[1], 4):
        ys = np.where(foam_active[:, x] > 0)[0]
        if len(ys) >= 3:
            top_points.append(ys[0])

    if len(top_points) < 5:
        return None

    return int(np.percentile(top_points, FOAM_TOP_PERCENTILE))


def detect_foam_thickness(frame):
    """
    Pipeline actual:
    ROI -> grayscale/CLAHE/blur -> Sobel/Otsu pentru nivel lichid ->
    segmentare culoare pentru crusta si spuma -> invatare crusta persistenta ->
    eliminare crusta -> pastrare spuma conectata la lichid -> grosime spuma.
    Returneaza grosimea stratului de spuma in pixeli.
    """
    global roi_coords

    if roi_coords is None:
        roi_coords = auto_calibrate_roi_from_frame(frame)

    y_start, y_end, x_start, x_end = roi_coords
    processed = preprocess_fermentation_frame(frame, roi_coords)

    if processed is None:
        return 0

    current_crust_mask = update_crust_model(processed)
    edge_bin = processed["edge_bin"].copy()

    if ENABLE_CRUST_FILTER and current_crust_mask is not None:
        crust_mask_inv = cv2.bitwise_not(current_crust_mask)
        edge_bin = cv2.bitwise_and(edge_bin, edge_bin, mask=crust_mask_inv)

    row_sum = np.sum(edge_bin, axis=1) / 255
    threshold_pixels = (x_end - x_start) * 0.25
    candidates = np.where(row_sum > threshold_pixels)[0]

    if len(candidates) < 5:
        return 0

    y_liquid_roi = candidates[-1]
    y_liquid = y_liquid_roi + y_start

    foam_active_initial = clean_active_foam_mask(processed["foam_candidate"], current_crust_mask)
    foam_active = keep_foam_connected_to_liquid(foam_active_initial, y_liquid_roi)
    foam_top_roi = estimate_foam_top_from_mask(foam_active)

    if foam_top_roi is None:
        return 0

    y_foam = foam_top_roi + y_start

    if y_foam >= y_liquid:
        return 0

    return y_liquid - y_foam


# =========================
# SIGNAL PROCESSING
# =========================
def rolling_median(values, kernel_size):
    if len(values) == 0:
        return values

    kernel_size = min(kernel_size, len(values))
    if kernel_size % 2 == 0:
        kernel_size -= 1
    if kernel_size < 3:
        return values.copy()

    half = kernel_size // 2
    filtered = np.zeros_like(values, dtype=float)

    for i in range(len(values)):
        start = max(0, i - half)
        end = min(len(values), i + half + 1)
        filtered[i] = np.median(values[start:end])

    return filtered


def smooth_signal(raw_values):
    values = np.array(raw_values, dtype=float)

    if len(values) == 0:
        return values, values

    min_v = np.min(values)
    max_v = np.max(values)

    if max_v == min_v:
        normalized = np.zeros_like(values)
    else:
        normalized = 100 * (values - min_v) / (max_v - min_v)

    corrected = rolling_median(normalized, kernel_size=51)

    for i in range(1, len(corrected)):
        jump = corrected[i] - corrected[i - 1]
        limit = 2.0
        if abs(jump) > limit:
            corrected[i] = corrected[i - 1] + np.sign(jump) * limit

    window = min(101, len(corrected))
    if window < 3:
        smoothed = corrected.copy()
    else:
        smoothed = np.convolve(corrected, np.ones(window) / window, mode="same")
        if len(smoothed) > 1:
            smoothed[-1] = smoothed[-2]

    return corrected, smoothed


# =========================
# CAPTURE TASK
# =========================
def capture_task():
    os.makedirs(IMAGES_DIR, exist_ok=True)
    next_capture_time = time.monotonic() + CAPTURE_INTERVAL_SECONDS

    while True:
        if AUTO_LIGHTS_FOR_CAPTURE:
            pre_light_time = next_capture_time - LIGHT_PRE_CAPTURE_SECONDS
            time.sleep(max(0, pre_light_time - time.monotonic()))

            lights_were_auto_enabled = prepare_lights_for_capture()
            time.sleep(max(0, next_capture_time - time.monotonic()))
        else:
            lights_were_auto_enabled = False
            time.sleep(max(0, next_capture_time - time.monotonic()))

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        frame, brightness = capture_valid_frame()

        if frame is None:
            print(f"[WARN] {timestamp} nu s-a putut citi niciun cadru din camera.")
            if lights_were_auto_enabled:
                time.sleep(LIGHT_POST_CAPTURE_SECONDS)
                finish_lights_after_capture(True)

            next_capture_time += CAPTURE_INTERVAL_SECONDS
            if next_capture_time <= time.monotonic():
                next_capture_time = time.monotonic() + CAPTURE_INTERVAL_SECONDS
            continue

        if brightness < CAPTURE_MIN_BRIGHTNESS:
            rejected_path = os.path.join(IMAGES_DIR, f"respins_{timestamp}.jpg")
            cv2.imwrite(rejected_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            print(
                f"[RESPINS] {timestamp} cadru prea intunecat "
                f"({brightness:.2f}). Salvat separat: {rejected_path}"
            )

            if lights_were_auto_enabled:
                time.sleep(LIGHT_POST_CAPTURE_SECONDS)
                finish_lights_after_capture(True)

            next_capture_time += CAPTURE_INTERVAL_SECONDS
            if next_capture_time <= time.monotonic():
                next_capture_time = time.monotonic() + CAPTURE_INTERVAL_SECONDS
            continue

        foam_thickness = detect_foam_thickness(frame)

        img_path = os.path.join(IMAGES_DIR, f"img_{timestamp}.jpg")
        cv2.imwrite(img_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])

        with data_lock:
            file_exists = os.path.isfile(DATASET_PATH)

            with open(DATASET_PATH, "a", newline="") as f:
                writer = csv.writer(f)

                if not file_exists:
                    writer.writerow(["timestamp", "foam_thickness_px"])

                writer.writerow([timestamp, foam_thickness])

        print(
            f"[DATA] {timestamp} grosime spuma: {foam_thickness:.2f}px "
            f"(luminozitate: {brightness:.2f})"
        )

        if lights_were_auto_enabled:
            time.sleep(LIGHT_POST_CAPTURE_SECONDS)
            finish_lights_after_capture(True)

        next_capture_time += CAPTURE_INTERVAL_SECONDS
        if next_capture_time <= time.monotonic():
            next_capture_time = time.monotonic() + CAPTURE_INTERVAL_SECONDS


# =========================
# GRAPH + ANALYSIS
# =========================
def load_dataset():
    timestamps = []
    values = []

    if not os.path.exists(DATASET_PATH):
        return timestamps, values

    with data_lock:
        with open(DATASET_PATH) as f:
            reader = csv.reader(f)
            next(reader, None)

            for row in reader:
                try:
                    ts = datetime.strptime(row[0], "%Y-%m-%d_%H-%M-%S")
                    val = float(row[1])
                    timestamps.append(ts)
                    values.append(val)
                except Exception:
                    continue

    return timestamps, values


def generate_graph():
    timestamps, values = load_dataset()

    if len(values) < 5:
        return None

    corrected, smoothed = smooth_signal(values)

    peak_i = int(np.argmax(smoothed))
    peak = smoothed[peak_i]

    if peak > 0 and smoothed[-1] < peak * 0.4:
        print("[ALERT] Fermentatia s-a incheiat!")

    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, corrected, alpha=0.3, label="Date brute corectate de inertie")
    plt.plot(timestamps, smoothed, linewidth=3, color="darkorange", label="Evolutie reala smoothed")
    plt.scatter(timestamps[peak_i], peak, color="red", s=80, zorder=5, label="Nivel maxim spuma")
    plt.annotate(
        "Maxim spuma",
        xy=(timestamps[peak_i], peak),
        xytext=(10, 12),
        textcoords="offset points",
        arrowprops=dict(arrowstyle="->", color="red", linewidth=1.2),
        color="red",
        fontsize=10,
    )

    plt.title("Evolutia stratului de spuma (Auto-ROI & filtrare semnal)")
    plt.xlabel("Timp")
    plt.ylabel("Nivel relativ spuma (%)")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.xticks(rotation=45)
    plt.tight_layout()

    os.makedirs("static", exist_ok=True)
    plt.savefig(GRAPH_PATH, dpi=130)
    plt.close()

    return GRAPH_PATH


def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = float(f.read()) / 1000
        return round(temp, 1)
    except Exception as e:
        print("Eroare temperatura:", e)
        return 0


# =========================
# ROUTES
# =========================
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/status")
def status():
    temp = get_cpu_temp()
    stare_stm = "Conectat" if (ser and ser.is_open) else "Deconectat"
    stare_reactor = "Conectat" if (reactor_ser and reactor_ser.is_open) else "Deconectat"

    timestamps, values = load_dataset()
    last_foam = values[-1] if values else 0

    with reactor_lock:
        reactor_snapshot = latest_reactor_data.copy()

    return jsonify({
        "temperatura": temp,
        "conexiune": stare_stm,
        "conexiune_reactor": stare_reactor,
        "port_reactor": reactor_active_port,
        "reactor_octeti_primiti": reactor_bytes_received,
        "reactor_ultima_valoare_valida": reactor_last_valid_at,
        "reactor_ultima_eroare": reactor_last_error,
        "roi_calibrat": roi_coords is not None,
        "crusta_invatata": crust_mask is not None,
        "lumini": latest_light_state,
        "lumini_auto_capture": AUTO_LIGHTS_FOR_CAPTURE,
        "lumini_pre_capture_secunde": LIGHT_PRE_CAPTURE_SECONDS,
        "lumini_post_capture_secunde": LIGHT_POST_CAPTURE_SECONDS,
        "numar_masuratori": len(values),
        "ultima_grosime_spuma_px": last_foam,
        "reactor": reactor_snapshot,
    })


@app.route("/comanda/<tip>")
def comanda_hardware(tip):
    command = STM_COMMANDS.get(tip.lower())

    if command is None:
        return f"Comanda necunoscuta: {tip}", 400

    ok, response = send_stm_command(command)
    if not ok:
        return response, 503

    return response


@app.route("/lumini/status")
def lumini_status():
    ok, response = send_stm_command("STATUS")
    return jsonify({
        "connected": ok,
        "state": latest_light_state,
        "response": response,
    }), 200 if ok else 503


@app.route("/graph")
def graph():
    path = generate_graph()
    if path is None:
        return "No data"
    return send_file(path, mimetype="image/png")


@app.route("/reactor_data")
def reactor_data():
    with reactor_lock:
        data = latest_reactor_data.copy()

    if not data:
        return jsonify({
            "connected": reactor_ser is not None and reactor_ser.is_open,
            "message": "Nu exista inca date de la reactor",
            "data": None,
        })

    return jsonify({
        "connected": reactor_ser is not None and reactor_ser.is_open,
        "data": data,
    })


@app.route("/reactor_history")
def reactor_history():
    try:
        limit = int(request.args.get("limit", 200))
    except ValueError:
        limit = 200

    limit = max(1, min(limit, 1000))
    history = load_reactor_history(limit)

    return jsonify({
        "count": len(history),
        "data": history,
    })


@app.route("/reactor_serial_status")
def reactor_serial_status():
    """Diagnostic detaliat pentru verificarea legaturii seriale cu bioreactorul."""
    get_reactor_candidate_ports()

    return jsonify({
        "connected": reactor_ser is not None and reactor_ser.is_open,
        "active_port": reactor_active_port,
        "configured_port": reactor_preferred_port,
        "baudrate": REACTOR_BAUDRATE,
        "format": "8N1, fara handshake",
        "available_ports": reactor_available_ports,
        "bytes_received": reactor_bytes_received,
        "lines_received": reactor_lines_received,
        "valid_rows": reactor_valid_rows,
        "transport_mode": reactor_transport_mode,
        "last_byte_at": reactor_last_byte_at,
        "last_valid_at": reactor_last_valid_at,
        "last_raw_line": reactor_last_raw_line,
        "last_raw_hex": reactor_last_raw_hex,
        "raw_tail_hex": bytes(reactor_raw_tail).hex(" "),
        "raw_tail_text": bytes(reactor_raw_tail).decode("latin-1", errors="replace"),
        "raw_log_path": REACTOR_RAW_LOG_PATH,
        "last_error": reactor_last_error,
        "recent_rejected_lines": list(reactor_rejected_lines),
    })


@app.route("/reactor_raw_log")
def reactor_raw_log():
    if not os.path.exists(REACTOR_RAW_LOG_PATH):
        return "Nu exista inca date seriale brute", 404

    return send_file(
        REACTOR_RAW_LOG_PATH,
        mimetype="application/octet-stream",
        as_attachment=True,
        download_name="reactor_serial_raw.bin",
    )


@app.route("/reactor_reconnect")
def reactor_reconnect():
    """Inchide portul curent; firul serial il va redeschide automat."""
    global reactor_ser, reactor_active_port, reactor_preferred_port

    requested_port = request.args.get("port", "").strip()
    if requested_port:
        if not requested_port.startswith("/dev/"):
            return jsonify({
                "ok": False,
                "message": "Port invalid. Exemplu acceptat: /dev/ttyUSB0",
            }), 400
        reactor_preferred_port = requested_port

    try:
        if reactor_ser is not None:
            reactor_ser.close()
    except Exception:
        pass

    reactor_ser = None
    reactor_active_port = None
    return jsonify({
        "ok": True,
        "message": "Reconectarea seriala a fost solicitata",
        "preferred_port": reactor_preferred_port,
    })


@app.route("/reset")
def reset_data():
    global roi_coords, crust_mask

    if request.args.get("confirm") != "yes":
        return "Reset blocat. Foloseste /reset?confirm=yes"

    try:
        with data_lock:
            if os.path.exists(DATASET_PATH):
                os.remove(DATASET_PATH)

        roi_coords = None
        crust_mask = None
        crust_edge_history.clear()
        crust_color_history.clear()
        print("[RESET] Date sterse si ROI resetat")
        return "RESET OK"

    except Exception as e:
        return f"Eroare reset: {e}"


@app.route("/reset_reactor")
def reset_reactor_data():
    global latest_reactor_data

    if request.args.get("confirm") != "yes":
        return "Reset bioreactor blocat. Foloseste /reset_reactor?confirm=yes"

    try:
        with reactor_lock:
            if os.path.exists(REACTOR_DATASET_PATH):
                os.remove(REACTOR_DATASET_PATH)
            latest_reactor_data = {}

        print("[RESET] Date bioreactor sterse")
        return "RESET BIOREACTOR OK"

    except Exception as e:
        return f"Eroare reset bioreactor: {e}"


@app.route("/debug_reactor_once")
def debug_reactor_once():
    global latest_reactor_data

    data = generate_debug_reactor_data()

    with reactor_lock:
        latest_reactor_data = data

    save_reactor_data(data)
    return jsonify({"message": "Citire simulata adaugata", "data": data})


@app.route("/debug_parse_reactor")
def debug_parse_reactor():
    global latest_reactor_data

    line = request.args.get("line", "")
    data = parse_reactor_line(line)

    if data is None:
        return jsonify({"ok": False, "message": "Randul nu a putut fi parsat", "line": line}), 400

    with reactor_lock:
        latest_reactor_data = data

    save_reactor_data(data)
    return jsonify({"ok": True, "data": data})


# =========================
# MAIN
# =========================
if __name__ == "__main__":
    threading.Thread(target=capture_task, daemon=True).start()
    threading.Thread(target=reactor_serial_task, daemon=True).start()
    app.run(host="0.0.0.0", port=80, debug=True, threaded=True, use_reloader=False)
