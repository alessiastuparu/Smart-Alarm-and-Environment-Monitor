"""
Sunrise Alarm - Raspberry Pi Pico 2W (MicroPython)
===================================================
GP0 = UART0 TX  ->  STM32 PA10 (USART1 RX)
GP1 = UART0 RX  <-  STM32 PA9  (USART1 TX)

UART protocol (must match stm32-core/src/main.rs):
  Pico -> STM32:
    SET_ALARM:HH:MM\n
    SNOOZE\n
    DISABLE\n
    SYNC_TIME:HH:MM:SS\n

  STM32 -> Pico (every ~2 s):
    STATUS:TIME=12:34:56,TEMP=23.4,HUM=58,ALARM=07:30,STATE=IDLE,LED_PHASE=0,LED_PROG=0\n

Connect to WiFi "SunriseAlarm" (no password), then open http://192.168.4.1
"""

import network
import socket
import time
import machine
import json

# ── Config ────────────────────────────────────────────────────────────────────
WIFI_SSID = "SunriseAlarm"

# ── UART to STM32 ─────────────────────────────────────────────────────────────
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))
uart.init(bits=8, parity=None, stop=1)

# ── Shared state (updated by poll_uart on every request cycle) ────────────────
state = {
    "time":      "--:--:--",
    "date":      "",
    "temp":      "--.-",
    "hum":       "--",
    "alarm":     "--:--",
    "status":    "IDLE",
    "led_phase": 0,
    "led_prog":  0,
    "remaining": "--",
}

uart_buf = b""

# ── UART helpers ──────────────────────────────────────────────────────────────
def uart_send(cmd):
    uart.write((cmd + "\n").encode())
    print("[UART TX]", cmd)

def poll_uart():
    global uart_buf
    if uart.any():
        chunk = uart.read(uart.any())
        if chunk:
            uart_buf += chunk
            while b"\n" in uart_buf:
                line, uart_buf = uart_buf.split(b"\n", 1)
                parse_status(line.decode("utf-8", "ignore").strip())

def parse_status(line):
    # Expects: STATUS:TIME=12:34:56,TEMP=23.4,HUM=58,ALARM=07:30,STATE=IDLE,LED_PHASE=0,LED_PROG=0
    try:
        if not line.startswith("STATUS:"):
            return
        payload = line[7:]
        fields = {}
        for part in payload.split(","):
            k, _, v = part.partition("=")
            fields[k.strip()] = v.strip()
        state["time"]      = fields.get("TIME",      state["time"])
        state["temp"]      = fields.get("TEMP",      state["temp"])
        state["hum"]       = fields.get("HUM",       state["hum"])
        state["alarm"]     = fields.get("ALARM",     state["alarm"])
        state["status"]    = fields.get("STATE",     state["status"])
        state["led_phase"] = int(fields.get("LED_PHASE", str(state["led_phase"])))
        state["led_prog"]  = int(fields.get("LED_PROG",  str(state["led_prog"])))
        print("[UART RX] parsed:", state["time"], state["status"])
    except Exception as e:
        print("[parse_status error]", e)

# ── Wi-Fi ─────────────────────────────────────────────────────────────────────
def setup_wifi():
    ap = network.WLAN(network.AP_IF)
    ap.active(False)
    time.sleep(0.5)
    ap.active(True)
    ap.config(ssid=WIFI_SSID, security=0)   # open — no password
    timeout = 10
    while not ap.active() and timeout > 0:
        time.sleep(0.5)
        timeout -= 1
    ip = ap.ifconfig()[0]
    print("[AP] SSID:", WIFI_SSID, "  IP:", ip)
    return ip

# ── HTTP helpers ──────────────────────────────────────────────────────────────
def send_response(conn, code, content_type, body):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + str(code) + " OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + str(len(body)) + "\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n"
    )
    conn.sendall(header.encode() + body)

def read_request(conn):
    data = b""
    conn.settimeout(3.0)
    try:
        while True:
            chunk = conn.recv(512)
            if not chunk:
                break
            data += chunk
            if b"\r\n\r\n" in data:
                header_end = data.index(b"\r\n\r\n") + 4
                header_str = data[:header_end].decode("utf-8", "ignore")
                content_length = 0
                for line in header_str.split("\r\n"):
                    if line.lower().startswith("content-length:"):
                        try:
                            content_length = int(line.split(":")[1].strip())
                        except Exception:
                            pass
                body_bytes = data[header_end:]
                while len(body_bytes) < content_length:
                    more = conn.recv(256)
                    if not more:
                        break
                    body_bytes += more
                data = data[:header_end] + body_bytes
                break
    except Exception:
        pass
    try:
        lines = data.decode("utf-8", "ignore").split("\r\n")
        parts = lines[0].split(" ")
        method = parts[0] if parts else "GET"
        path   = parts[1] if len(parts) > 1 else "/"
        body   = lines[-1] if lines else ""
        return method, path, body
    except Exception:
        return "GET", "/", ""

# ── Route handlers ────────────────────────────────────────────────────────────
def handle_status(conn):
    poll_uart()
    send_response(conn, 200, "application/json", json.dumps(state))

def handle_set_alarm(conn, body):
    # dashboard.html sends: {"time": "HH:MM"}
    try:
        data = json.loads(body)
        t = data.get("time", "")          # "HH:MM"
        if len(t) == 5 and t[2] == ":":
            uart_send("SET_ALARM:" + t)   # STM32 expects SET_ALARM:HH:MM
            send_response(conn, 200, "application/json", '{"ok":true}')
        else:
            send_response(conn, 400, "application/json", '{"ok":false}')
    except Exception as e:
        print("[set_alarm error]", e)
        send_response(conn, 400, "application/json", '{"ok":false}')

def handle_snooze(conn):
    uart_send("SNOOZE")
    send_response(conn, 200, "application/json", '{"ok":true}')

def handle_disable(conn):
    uart_send("DISABLE")
    send_response(conn, 200, "application/json", '{"ok":true}')

def handle_sync_time(conn, body):
    # dashboard.html sends: {"time": "HH:MM:SS"}
    try:
        data = json.loads(body)
        t = data.get("time", "")          # "HH:MM:SS"
        if len(t) == 8:
            uart_send("SYNC_TIME:" + t)   # STM32 expects SYNC_TIME:HH:MM:SS
            send_response(conn, 200, "application/json", '{"ok":true}')
        else:
            send_response(conn, 400, "application/json", '{"ok":false}')
    except Exception:
        send_response(conn, 400, "application/json", '{"ok":false}')

def load_html():
    try:
        with open("dashboard.html", "rb") as f:
            return f.read()
    except Exception:
        return b"<h1>dashboard.html missing — copy it to Pico flash</h1>"

# ── Main server loop ──────────────────────────────────────────────────────────
def run_server(ip):
    html_cache = load_html()
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, 80))
    s.listen(4)
    print("[HTTP] http://" + ip + "/")

    while True:
        poll_uart()
        try:
            s.settimeout(0.1)
            try:
                conn, addr = s.accept()
            except OSError:
                continue  # timeout — loop back and poll UART again

            poll_uart()
            method, path, body = read_request(conn)
            print("[HTTP]", method, path)

            if path == "/" or path == "/index.html":
                send_response(conn, 200, "text/html; charset=utf-8", html_cache)
            elif path == "/status":
                handle_status(conn)
            elif path == "/set_alarm" and method == "POST":
                handle_set_alarm(conn, body)
            elif path == "/snooze" and method == "POST":
                handle_snooze(conn)
            elif path == "/disable" and method == "POST":
                handle_disable(conn)
            elif path == "/sync_time" and method == "POST":
                handle_sync_time(conn, body)
            else:
                send_response(conn, 404, "text/plain", "Not found")

            conn.close()

        except Exception as e:
            print("[server error]", e)
            try:
                conn.close()
            except Exception:
                pass

# ── Entry point ───────────────────────────────────────────────────────────────
ip = setup_wifi()
time.sleep(1)
run_server(ip)