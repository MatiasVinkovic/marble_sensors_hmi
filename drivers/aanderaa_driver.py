import serial
import time

SCALAR_FIELDS = {
    "Pitch", "Roll", "Heading",
    "StDev Pitch", "StDev Roll", "StDev Heading",
    "Significant Wave Height Hm0",
    "Wave Height Wind Hm0", "Wave Height Swell Hm0",
    "Wave Height H1/3", "Wave Height Hmax",
    "Wave Mean Period Tz", "Wave Mean Period Tm02",
    "Wave Peak Period Wind", "Wave Peak Period Swell",
    "Wave Peak Direction", "Wave Peak Direction Wind", "Wave Peak Direction Swell",
    "Wave Mean Direction", "Mean Spreading Angle",
    "Input Voltage", "Input Current", "Memory Used",
}


def read_all(conn: serial.Serial, seconds: float) -> bytes:
    buf, deadline, last_rx = b'', time.time() + seconds, time.time()
    while time.time() < deadline:
        n = conn.in_waiting
        if n:
            buf += conn.read(n)
            last_rx = time.time()
        else:
            if buf and (time.time() - last_rx) > 0.5:
                break
            time.sleep(0.01)
    return buf


def sst(conn: serial.Serial, command: str, wait: float = 3.0) -> str:
    conn.reset_input_buffer()
    conn.write((command + '\r\n').encode('ascii'))
    raw = read_all(conn, wait)
    return raw.decode('ascii', errors='replace') if raw else ''


def parse_do_output(text: str) -> dict:
    """Parse la réponse DO_OUTPUT SST → dict {nom: (valeur_str, unité)}."""
    data = {}
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith('MEASUREMENT'):
            continue
        parts = line.split('\t')
        i, current_name, current_unit = 3, None, ''
        while i < len(parts):
            p = parts[i].strip()
            if p.startswith('*'):
                raw = p[1:]
                if '[' in raw and ']' in raw:
                    bracket = raw.index('[')
                    current_name = raw[:bracket].strip()
                    current_unit = raw[bracket + 1:raw.index(']')]
                else:
                    current_name = raw.strip()
                    current_unit = ''
                i += 1
            elif current_name and p:
                if current_name not in data:
                    data[current_name] = (p, current_unit)
                i += 1
            else:
                i += 1
    return data


def fmt(raw_val: str, unit: str) -> str:
    try:
        f = float(raw_val)
        if unit in ('Bytes', '') and f == int(f):
            return str(int(f))
        if 'Deg' in unit or unit in ('deg', '°'):
            return f'{f:.1f}'
        if unit == 'm':
            return f'{f:.3f}'
        if unit == 's':
            return f'{f:.2f}'
        if unit in ('V', 'mA'):
            return f'{f:.2f}'
        if abs(f) >= 1000 or (abs(f) < 0.001 and f != 0):
            return f'{f:.3e}'
        return f'{f:.3f}'
    except ValueError:
        return raw_val
