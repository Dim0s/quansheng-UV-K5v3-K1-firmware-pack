import os
import struct
import logging
import time
import serial

from chirp import chirp_common, directory, bitwise, memmap, errors, util
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueBoolean, RadioSettingValueList, \
    RadioSettingValueString, RadioSettings

LOG = logging.getLogger(__name__)

DEBUG_SHOW_OBFUSCATED_COMMANDS = True
DEBUG_SHOW_MEMORY_ACTIONS = True
APP_REQUEST_RETRIES = 3
SESSION_TIMESTAMP = b"\x6a\x39\x57\x64"
CONFIG_ACCESS_PRE_DELAY = 0.08
CONFIG_ACCESS_POST_DELAY = 0.08
BANK_NAME_START = 0x14C0
BANK_NAME_SIZE = 8
BANK_NAME_MAGIC = b"BNKNAME1"
BANK_NAME_MAGIC_START = BANK_NAME_START + (16 * BANK_NAME_SIZE)
PLUGIN_LOG_PATH = os.path.join(os.path.dirname(__file__), "chirp_ijv_v4_serial.log")

def _setup_plugin_logging():
    if getattr(_setup_plugin_logging, "_done", False):
        return
    _setup_plugin_logging._done = True

    try:
        handler = logging.FileHandler(PLUGIN_LOG_PATH, mode="a", encoding="utf-8")
        handler.setLevel(logging.DEBUG)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)s %(name)s: %(message)s"))
        root = logging.getLogger()
        root.addHandler(handler)
        if root.level > logging.DEBUG:
            root.setLevel(logging.DEBUG)
        LOG.debug("CHIRP IJV plugin logging enabled: %s", PLUGIN_LOG_PATH)
    except Exception as e:
        LOG.warning("Unable to enable plugin file logging: %s", e)

_setup_plugin_logging()

# ============================================================
# MEM_1 - campi minimi dell'area fissa richiesti dal driver memory-only
# ============================================================
MEM_1 = """
#seekto 0x000B;
u8 mem_bank;

#seekto 0x14C0;
struct
{
    char name[8];
} bank_name[16];

#seekto 0x5E80;
struct
{
    char name[8];
} list_name[16];
"""

# ============================================================
# MEM_CHAN â€” struttura canali (0x2000-0x5E7F, 500 canali Ã— 32B)
# Adattata da "3.50 - Banco 0.py" per 500 canali
# ============================================================
MEM_CHAN = """
    #seekto 0x2000;

    struct
    {
      // ---------------rec 1 + 2
      char name[10];
      u8 code_sel0:4,
         code_sel1:4;
      u8 code_sel2:4,
         code_sel3:4;
      u8 code_sel4:4,
         code_sel5:4;
      u8 code_sel6:4,
         code_sel7:4;
      u8 code_sel8:4,
         code_sel9:4;
      u8 group:4,
         band:4;

      // ---------------rec 3
      ul32 freq;
      ul32 offset;

      // ---------------rec 4
      u8 rxcode;
      u8 txcode;

      u8 tx_codetype:4,
         rx_codetype:4;

      u8 txlock:1,
         writeprot:1,
         enablescan:1,
         modulation:3,
         shift:2;

      u8 busylock:1,
         txpower:2,
         bw:4,
         reverse:1;
       u8 no_used4:2,
         scrambler:1,
         compander:2,
         agcmode:3;

      u8 step_squelch;
      u8 dig_ptt;

    } channel[500];
"""

# MEM_FORMAT unificato: area fissa + array canali
MEM_FORMAT = MEM_1 + MEM_CHAN

# ============================================================
# Costanti di layout
# ============================================================
CHAN_MAX       = 500    # canali per banco
MEM_SIZE       = 0x6000 # immagine completa: area canali + nomi gruppo del banco attivo
START_MEM      = 0x2000 # inizio area canali
END_MEM        = 0x6000 # fine area canali + gruppi/nome banco del banco 0
MEM_BLOCK      = 0x20   # blocco per scrittura/lettura VCP: 32 byte → risposta 48 byte = 1 pacchetto USB
                        # (128 byte → 144 byte = 3 pacchetti USB → instabile su VCP nativo PY32F071)

# ============================================================
# Costanti protocollo
# ============================================================
OFFSET_PLUS  = 0b01
OFFSET_MINUS = 0b10

POWER_LOW    = 0b00
POWER_MEDIUM = 0b01
POWER_HIGH   = 0b10

# ============================================================
# Liste valori usate dalla gestione memorie
# ============================================================

BANDWIDTH_LIST = ["W 26k","W 23k","W 20k","W 17k","W 14k","W.12k",
                  "N 10k","N. 9k","U  7k","U  6k"]

MODULATION_LIST = ["FM","AM","USB","CW","WFM","DIG","NAM"] #ordine di come vengono scritti in eprom
#MODULATION_LIST = ["FM", "AM", "WFM", "USB", "CW", "NAM", "DIG"]
PTTID_LIST = ["OFF", "CALL ID", "SEL CALL", "CODE BEGIN", "CODE END",
              "CODE BEG+END", "ROGER Single", "ROGER 2Tones",
              "Apollo Quindar"]

UVK5_POWER_LEVELS = [chirp_common.PowerLevel("Low",  watts=1.00),
                     chirp_common.PowerLevel("Med",  watts=2.50),
                     chirp_common.PowerLevel("High", watts=5.00)]

DIGITAL_CODE_LIST = ["OFF","DTMF","ZVEI1","ZVEI2","CCIR-1","CCIR-1F","ZVEI3","CCIR20","EEA"]

SQUELCH_LIST = ["Squelch 0","Squelch 1","Squelch 2","Squelch 3","Squelch 4",
                "Squelch 5","Squelch 6","Squelch 7","Squelch 8","Squelch 9","NO RX"]

COMPANDER_LIST = ["OFF", "TX", "RX", "RX/TX"]

SKIP_VALUES = ["", "S"]

# steps
STEPS = [0.01, 0.05, 0.10, 0.50, 1.00, 2.50, 5.00, 6.25, 8.33, 9.00,
         10.00, 12.50, 20.00, 25.00, 50.00, 100.00]

AGC_MODE = ["AUTO","MAN","FAST","NORM","SLOW"]

TMODES   = ["", "Tone", "DTCS", "DTCS"]

CTCSS_TONES = [
    67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4,
    88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9,
    114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2,
    151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3, 173.8,
    177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5,
    203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8,
    250.3, 254.1,
]

DTCS_CODES = [
    23,  25,  26,  31,  32,  36,  43,  47,  51,  53,  54,
    65,  71,  72,  73,  74,  114, 115, 116, 122, 125, 131,
    132, 134, 143, 145, 152, 155, 156, 162, 165, 172, 174,
    205, 212, 223, 225, 226, 243, 244, 245, 246, 251, 252,
    255, 261, 263, 265, 266, 271, 274, 306, 311, 315, 325,
    331, 332, 343, 346, 351, 356, 364, 365, 371, 411, 412,
    413, 423, 431, 432, 445, 446, 452, 454, 455, 462, 464,
    465, 466, 503, 506, 516, 523, 526, 532, 546, 565, 606,
    612, 624, 627, 631, 632, 654, 662, 664, 703, 712, 723,
    731, 732, 734, 743, 754
]

BANDS = {
    0: [13.0,   107.9999],
    1: [108.0,  136.9999],
    2: [137.0,  173.9990],
    3: [174.0,  349.9999],
    4: [350.0,  399.9999],
    5: [400.0,  469.9999],
    6: [470.0, 1299.9999]
}

SPECIALS = {}

GROUP_FALLBACK_LIST = ["ALL"] + ["Group %d" % i for i in range(1, 16)]

# ============================================================
# Funzioni di utilitÃ 
# ============================================================

def _clean_ascii_field(value):
    return _safe_text(value).strip("\x20\x00\xff")


def _sanitize_ascii_name(value, default_name, max_len=8):
    text = _safe_text(value)
    clean = "".join(ch for ch in text if 32 <= ord(ch) <= 126).strip()
    clean = clean[:max_len]
    if not clean:
        clean = default_name[:max_len]
    return clean.ljust(max_len)


def build_group_list(memobj):
    group_list = list(GROUP_FALLBACK_LIST)
    if memobj is None or not hasattr(memobj, "list_name"):
        return group_list

    for i in range(16):
        name = _clean_ascii_field(memobj.list_name[i].name)
        group_list[i] = name if name else GROUP_FALLBACK_LIST[i]

    return group_list


def get_bank_index(memobj, fallback=None):
    if memobj is None or not hasattr(memobj, "mem_bank"):
        return fallback

    try:
        bank = int(memobj.mem_bank)
    except Exception:
        LOG.exception("Unable to read mem_bank from mmap")
        return fallback

    return bank if 0 <= bank <= 15 else fallback


def get_bank_name(memobj, bank_index=None):
    if bank_index is None:
        bank_index = 0

    if memobj is not None and hasattr(memobj, "bank_name") and 0 <= bank_index < 16:
        name = _clean_ascii_field(memobj.bank_name[bank_index].name)
    else:
        name = ""

    if name:
        return name

    return "BANK %02d" % bank_index

def _safe_text(value):
    try:
        if value is None:
            return ""
        text = str(value)
        if text is None:
            LOG.warning("safe_text got None string from %r", value)
            return ""
        return text
    except Exception:
        LOG.exception("safe_text exception for value=%r", value)
        return ""

#--------------------------------------------------------------------------------
# nibble â†’ ascii (per codici selettive)
def hexasc(data):
    res = data
    if res <= 9:
        return chr(res+48)
    elif data == 0xA: return "A"
    elif data == 0xB: return "B"
    elif data == 0xC: return "C"
    elif data == 0xD: return "D"
    elif data == 0xF: return "F"
    else:             return " "

#--------------------------------------------------------------------------------
# ascii â†’ nibble (per codici selettive)
def ascdec(data):
    if   data == "0": return 0
    elif data == "1": return 1
    elif data == "2": return 2
    elif data == "3": return 3
    elif data == "4": return 4
    elif data == "5": return 5
    elif data == "6": return 6
    elif data == "7": return 7
    elif data == "8": return 8
    elif data == "9": return 9
    elif data == "A": return 10
    elif data == "B": return 11
    elif data == "C": return 12
    elif data == "D": return 13
    elif data == "F": return 15
    else:             return 14

#--------------------------------------------------------------------------------
# obfuscation XOR (protocollo K5)
def xorarr(data: bytes):
    tbl = [22, 108, 20, 230, 46, 145, 13, 64, 33, 53, 213, 64, 19, 3, 233, 128]
    x = b""
    r = 0
    for byte in data:
        x += bytes([byte ^ tbl[r]])
        r = (r+1) % len(tbl)
    return x

#--------------------------------------------------------------------------------
def calculate_crc16_xmodem(data: bytes):
    poly = 0x1021
    crc  = 0x0
    for byte in data:
        crc = crc ^ (byte << 8)
        for i in range(8):
            crc = crc << 1
            if (crc & 0x10000):
                crc = (crc ^ poly) & 0xFFFF
    return crc & 0xFFFF

#--------------------------------------------------------------------------------
def _send_command(serport, data: bytes):
    """Invia un comando alla radio UV-K5"""
    LOG.debug("Sending command (unobfuscated) len=0x%4.4x:\n%s" %
              (len(data), util.hexprint(data)))

    crc     = calculate_crc16_xmodem(data)
    data2   = data + struct.pack("<H", crc)
    command = struct.pack(">HBB", 0xabcd, len(data), 0) + \
              xorarr(data2) + struct.pack(">H", 0xdcba)

    if DEBUG_SHOW_OBFUSCATED_COMMANDS:
        LOG.debug("Sending command (obfuscated):\n%s" % util.hexprint(command))
    try:
        result = serport.write(command)
    except Exception:
        raise errors.RadioError("Error writing data to radio")
    return result

#--------------------------------------------------------------------------------
def _receive_reply(serport):
    header = serport.read(4)
    if len(header) != 4:
        LOG.warning("Header short read: [%s] len=%i" % (util.hexprint(header), len(header)))
        raise errors.RadioError("Header short read")
    if header[0] != 0xAB or header[1] != 0xCD or header[3] != 0x00:
        LOG.warning("Bad response header: %s len=%i" % (util.hexprint(header), len(header)))
        raise errors.RadioError("Bad response header")

    cmd = serport.read(int(header[2]))
    if len(cmd) != int(header[2]):
        LOG.warning("Body short read: [%s] len=%i" % (util.hexprint(cmd), len(cmd)))
        raise errors.RadioError("Command body short read")

    footer = serport.read(4)
    if len(footer) != 4:
        LOG.warning("Footer short read: [%s] len=%i" % (util.hexprint(footer), len(footer)))
        raise errors.RadioError("Footer short read")
    if footer[2] != 0xDC or footer[3] != 0xBA:
        LOG.warning("Bad response footer: %s len=%i" % (util.hexprint(footer), len(footer)))
        raise errors.RadioError("Bad response footer")

    if DEBUG_SHOW_OBFUSCATED_COMMANDS:
        LOG.debug("Received reply (obfuscated) len=0x%4.4x:\n%s" % (len(cmd), util.hexprint(cmd)))

    cmd2 = xorarr(cmd)
    LOG.debug("Received reply (unobfuscated) len=0x%4.4x:\n%s" % (len(cmd2), util.hexprint(cmd2)))
    return cmd2

#--------------------------------------------------------------------------------
def _clear_serial_buffers(serport):
    try:
        serport.reset_input_buffer()
    except Exception:
        pass
    try:
        serport.reset_output_buffer()
    except Exception:
        pass

#--------------------------------------------------------------------------------
def _reopen_app_pipe(radio, timeout=1.0):
    oldpipe = radio.pipe
    port = getattr(oldpipe, "port", None)
    if not port:
        LOG.warning("radio.pipe has no port attribute; reusing existing pipe")
        return oldpipe

    LOG.debug("Reopening serial pipe on %s timeout=%s", port, timeout)
    try:
        oldpipe.close()
    except Exception:
        pass

    newpipe = serial.Serial()
    newpipe.port = port
    newpipe.baudrate = getattr(radio, "BAUD_RATE", 38400)
    newpipe.timeout = timeout
    newpipe.dsrdtr = False
    newpipe.rtscts = False
    try:
        newpipe.dtr = True
    except Exception:
        pass
    try:
        newpipe.rts = False
    except Exception:
        pass
    newpipe.open()
    time.sleep(0.2)
    _clear_serial_buffers(newpipe)
    radio.pipe = newpipe
    LOG.debug("Serial pipe reopened on %s baud=%s dtr=%s rts=%s timeout=%s",
              newpipe.port, newpipe.baudrate, getattr(newpipe, "dtr", None),
              getattr(newpipe, "rts", None), newpipe.timeout)
    return newpipe

#--------------------------------------------------------------------------------
def _request_radio(serport, payload: bytes, expected_cmd=None):
    last_error = None
    cmd_id = None
    if len(payload) >= 2:
        cmd_id = struct.unpack_from("<H", payload, 0)[0]
    for attempt in range(APP_REQUEST_RETRIES):
        try:
            LOG.debug("Request attempt %d/%d expected_cmd=%s payload_len=%d",
                      attempt + 1, APP_REQUEST_RETRIES,
                      "0x%04X" % expected_cmd if expected_cmd is not None else "None",
                      len(payload))
            if attempt > 0:
                time.sleep(0.15)
                _clear_serial_buffers(serport)
            if cmd_id == 0x0562:
                LOG.debug("Applying config-access pre-delay %.3fs", CONFIG_ACCESS_PRE_DELAY)
                time.sleep(CONFIG_ACCESS_PRE_DELAY)
            _send_command(serport, payload)
            reply = _receive_reply(serport)
            if cmd_id == 0x0562:
                LOG.debug("Applying config-access post-delay %.3fs", CONFIG_ACCESS_POST_DELAY)
                time.sleep(CONFIG_ACCESS_POST_DELAY)
            if expected_cmd is not None:
                cmd = struct.unpack_from("<H", reply, 0)[0]
                if cmd != expected_cmd:
                    raise errors.RadioError(
                        "Unexpected reply 0x%04X, expected 0x%04X" %
                        (cmd, expected_cmd))
            return reply
        except errors.RadioError as e:
            last_error = e
            LOG.warning("Request failed attempt %d/%d: %s",
                        attempt + 1, APP_REQUEST_RETRIES, e)

    if last_error is None:
        last_error = errors.RadioError("Unknown communication failure")
    raise errors.RadioError(
        "Communication failed after %d attempts: %s" %
        (APP_REQUEST_RETRIES, last_error))

#--------------------------------------------------------------------------------
def _getstring(data: bytes, begin, maxlen):
    end = min(begin + maxlen, len(data))
    chars = []
    for item in data[begin:end]:
        if isinstance(item, int):
            val = item
        else:
            text = str(item or "")
            if len(text) != 1:
                break
            val = ord(text)
        if val < 0x20 or val > 0x7E:
            break
        chars.append(chr(val))
    return "".join(chars)

#--------------------------------------------------------------------------------
def _supports_config_access(firmware_version):
    text = (firmware_version or "").strip().upper()
    return text.startswith("V-X")

#--------------------------------------------------------------------------------
# _sayhello() — Negozia la sessione con la radio e restituisce la versione firmware.
#
# FIX: il loop di retry originale non funzionava perché _receive_reply() lancia
# errors.RadioError in caso di timeout/risposta errata anziché restituire None.
# L'eccezione propagava fuori dal loop senza decrementare 'tries'.
#
# FIX: delay iniziale da 150ms prima del primo comando.
# Con usbser.sys (driver CDC nativo Windows), EscapeCommFunction(SETDTR) può
# ritornare prima che il trasferimento USB SetControlLineState sia completato
# dal firmware. Senza il delay, il firmware riceve l'hello, imposta il countdown
# di 6s (blocca la radio), ma non ha ancora dtr_enable=1 → risposta VCP droppata.
# 150ms garantisce che SetControlLineState sia processato prima dell'hello.
#--------------------------------------------------------------------------------
def _sayhello(serport):
    hellopacket = b"\x14\x05\x04\x00" + SESSION_TIMESTAMP

    # Attende che usbser.sys completi il SetControlLineState (DTR=1).
    # Senza questo delay il firmware riceve l'hello ma non può rispondere
    # via VCP perché dtr_enable è ancora 0 → risposta droppata → blocco radio.
    try:
        serport.dtr = True
    except Exception:
        pass
    time.sleep(0.20)

    tries = 5
    while tries > 0:
        LOG.debug("Sending hello packet (tentativo %d/5)" % (6 - tries))
        try:
            if tries < 5:
                _clear_serial_buffers(serport)
            o = _request_radio(serport, hellopacket, expected_cmd=0x0515)
            if o:
                firmware = _getstring(o, 4, 16)
                LOG.info("Found firmware: %s" % firmware)
                return firmware
        except errors.RadioError as e:
            LOG.warning("Hello tentativo fallito: %s" % str(e))
        tries -= 1

    LOG.warning("Failed to initialise radio after 5 attempts")
    raise errors.RadioError("Failed to initialize radio")

#--------------------------------------------------------------------------------
def _readmem(serport, offset, length):
    LOG.debug("Sending readmem offset=0x%4.4x len=0x%4.4x" % (offset, length))
    readmem = b"\x1b\x05\x08\x00" + \
              struct.pack("<HBB", offset, length, 0) + \
              SESSION_TIMESTAMP
    o = _request_radio(serport, readmem, expected_cmd=0x051c)
    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("readmem Received data len=0x%4.4x:\n%s" % (len(o), util.hexprint(o)))
    if len(o) < 8:
        raise errors.RadioError("Short readmem reply")
    if o[4] != (offset & 0xff) or o[5] != ((offset >> 8) & 0xff) or o[6] != length:
        raise errors.RadioError(
            "Unexpected readmem reply offset=0x%04X len=0x%02X" %
            (o[4] | (o[5] << 8), o[6]))
    return o[8:]

#--------------------------------------------------------------------------------
def _config_write_menu_bank(serport, bank):
    if bank < 0 or bank > 15:
        return False
    payload = b"\x62\x05\x05\x00" + bytes((1, 1, 0, 1, bank))
    o = _request_radio(serport, payload, expected_cmd=0x0563)
    return bool(o and len(o) >= 9 and o[0] == 0x63 and o[1] == 0x05 and o[4] == 1 and o[5] == 1 and o[6] == 0 and o[7] == 0)

#--------------------------------------------------------------------------------
def _config_read_menu_bank(serport):
    payload = b"\x62\x05\x04\x00" + bytes((0, 1, 0, 1))
    o = _request_radio(serport, payload, expected_cmd=0x0563)
    if not (o and len(o) >= 10 and o[0] == 0x63 and o[1] == 0x05):
        raise errors.RadioError("Bad response to config_read_menu_bank")
    if o[4] != 0 or o[5] != 1 or o[6] != 0 or o[7] != 0 or o[8] != 1:
        raise errors.RadioError("Unexpected response header to config_read_menu_bank")
    return o[9]

#--------------------------------------------------------------------------------
def _writemem(serport, data, offset):
    LOG.debug("Sending writemem offset=0x%4.4x len=0x%4.4x" % (offset, len(data)))
    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("writemem sent data offset=0x%4.4x len=0x%4.4x:\n%s" %
                  (offset, len(data), util.hexprint(data)))
    dlen     = len(data)
    writemem = b"\x1d\x05" + \
               struct.pack("<BBHBB", dlen+8, 0, offset, dlen, 1) + \
               SESSION_TIMESTAMP + data
    o = _request_radio(serport, writemem, expected_cmd=0x051e)
    LOG.debug("writemem Received data: %s len=%i" % (util.hexprint(o), len(o)))
    if (o[0] == 0x1e and
            o[4] == (offset & 0xff) and
            o[5] == (offset >> 8) & 0xff):
        return True
    else:
        LOG.warning("Bad data from writemem")
        raise errors.RadioError("Bad response to writemem")

#--------------------------------------------------------------------------------
# Scrittura differenziale: legge il blocco dalla flash e scrive SOLO se diverso.
#
# Ottimizzazione critica per l'upload CHIRP sul firmware IJV V3:
#   Il firmware PY32F071 esegue il sector erase (~300ms) + page program (~80ms)
#   della flash SPI PY25Q16 in modo bloccante (WaitWIP polling) per ogni settore
#   4KB che viene modificato. Scrivere blocchi identici causa erase inutili,
#   aumentando il tempo di upload e la pressione sul buffer UART (38400 baud).
#   Con questa funzione, se l'utente non ha modificato una zona di memoria,
#   quella zona non viene scritta â†’ zero erase inutili â†’ upload molto piÃ¹ veloce.
#
# Effetto pratico:
#   - se l'utente modifica pochi canali, vengono riscritti solo i blocchi davvero cambiati.
#--------------------------------------------------------------------------------
def _writemem_if_changed(serport, data, offset):
    dlen    = len(data)
    current = _readmem(serport, offset, dlen)   # legge il blocco attuale dalla flash
    if current and bytes(current) == bytes(data):
        LOG.debug("writemem skip (unchanged) offset=0x%4.4x" % offset)
        return True                              # identico â†’ nessuna scrittura, nessun erase
    return _writemem(serport, data, offset)      # diverso â†’ scrivi normalmente

#--------------------------------------------------------------------------------
def _resetradio(serport):
    resetpacket = b"\xdd\x05\x00\x00"
    _send_command(serport, resetpacket)

#--------------------------------------------------------------------------------
# Lettura EEPROM dalla radio
# Download: 0x0000-0x5FFF (immagine completa necessaria per leggere il banco attivo
#           e l'area canali/gruppi del banco corrente)
#--------------------------------------------------------------------------------
def do_download(radio):
    serport = _reopen_app_pipe(radio, timeout=1.0)
    LOG.debug("do_download started on port=%s log=%s", getattr(serport, "port", None), PLUGIN_LOG_PATH)
    status = chirp_common.Status()
    status.cur = 0
    status.max = MEM_SIZE
    status.msg = "Downloading from radio"
    radio.status_fn(status)

    eeprom = b""

    # Pulisce il buffer RX da eventuali byte residui (sessioni precedenti, reset radio).
    f = _sayhello(serport)
    if f:
        radio.FIRMWARE_VERSION = f
    else:
        raise errors.RadioError('Unable to determine firmware version')

    # 1) Legge il banco attivo dal firmware.
    if _supports_config_access(radio.FIRMWARE_VERSION):
        try:
            radio.ACTIVE_BANK = _config_read_menu_bank(serport)
        except Exception as e:
            LOG.warning("Unable to read active bank: %s", e)
            radio.ACTIVE_BANK = None
    else:
        radio.ACTIVE_BANK = None

    # 2) Se disponibile, forza il layer compatibile sul banco attivo appena letto.
    # 3) A questo punto 0x2000-0x5FFF espone gruppi e memorie del banco attivo.
    if _supports_config_access(radio.FIRMWARE_VERSION) and radio.ACTIVE_BANK is not None:
        try:
            _config_write_menu_bank(serport, radio.ACTIVE_BANK)
        except Exception as e:
            LOG.warning("Unable to select active bank before download: %s", e)

    addr = 0
    while addr < MEM_SIZE:
        o = _readmem(serport, addr, MEM_BLOCK)
        status.cur = addr
        radio.status_fn(status)
        if o and len(o) == MEM_BLOCK:
            eeprom += o
            addr   += MEM_BLOCK
        else:
            raise errors.RadioError("Memory download incomplete")

    return memmap.MemoryMapBytes(eeprom)

#--------------------------------------------------------------------------------
# Scrittura EEPROM sulla radio
# Build memory-only: upload della sola area canali del banco corrente
#   0x2000 - 0x6000  (500 canali + gruppi + nome banco)
#--------------------------------------------------------------------------------
def do_upload(radio):
    serport = _reopen_app_pipe(radio, timeout=1.0)
    LOG.debug("do_upload started on port=%s log=%s", getattr(serport, "port", None), PLUGIN_LOG_PATH)
    status = chirp_common.Status()
    status.cur = 0
    status.max = END_MEM - START_MEM
    status.msg = "Uploading Channels to radio"
    radio.status_fn(status)

    # Pulisce buffer RX da eventuali byte residui della sessione di download precedente.
    # Senza questo reset, bytes spuri nel buffer possono corrompere il parsing della risposta hello.
    f = _sayhello(serport)
    if f:
        radio.FIRMWARE_VERSION = f
    else:
        return False

    # Applica il banco attivo prima di scrivere gruppi e memorie del layer compatibile.
    if _supports_config_access(radio.FIRMWARE_VERSION):
        try:
            image_bank = radio.ACTIVE_BANK
            if image_bank is None and getattr(radio, "_memobj", None) is not None:
                image_bank = get_bank_index(radio._memobj, None)
            if image_bank is None:
                image_bank = radio.get_mmap()[0x0008 + 3]
            _config_write_menu_bank(serport, image_bank if image_bank <= 15 else 0)
        except Exception as e:
            LOG.warning("Unable to resync memory bank before upload: %s", e)
    else:
        LOG.debug("Skipping config-access bank resync for firmware '%s'", radio.FIRMWARE_VERSION)

    # Scrittura banco corrente (0x2000-0x5FFF)
    # Scrittura differenziale: skip dei blocchi invariati â†’ upload veloce
    # se solo pochi canali sono stati modificati rispetto al download precedente.
    addr = START_MEM
    while addr < END_MEM:
        o = radio.get_mmap()[addr:addr+MEM_BLOCK]
        _writemem_if_changed(serport, o, addr)
        status.cur = addr - START_MEM
        radio.status_fn(status)
        if o:
            addr += MEM_BLOCK
        else:
            raise errors.RadioError("Channel upload incomplete. If the error persists, run RESET ALL on the radio and try again.")

    status.msg = "Upload OK. If the radio shows incorrect data, run MENU -> RESET ALL on the radio."
    _resetradio(serport)
    return True

#--------------------------------------------------------------------------------
def _find_band(hz):
    mhz = hz / 1000000.0
    for a in BANDS:
        if mhz >= BANDS[a][0] and mhz <= BANDS[a][1]:
            return a
    return False

################################################################################################################################
################################################################################################################################

@directory.register
class UVK5Radio(chirp_common.CloneModeRadio):
    "IJV V4.0.4 (memory-only, 500 ch)"
    VENDOR   = "Quansheng"
    MODEL    = "IJV"
    VARIANT  = "V4.0.4 - Memory Only"
    BAUD_RATE = 38400
    NEEDS_COMPAT_SERIAL = False
    FIRMWARE_VERSION    = ""
    ACTIVE_BANK         = None
    ACTIVE_BANK_NAME    = ""
    _expanded_limits    = True

#--------------------------------------------------------------------------------
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank              = False
        rf.has_rx_dtcs           = True
        rf.has_ctone             = True
        rf.has_settings          = True
        rf.has_comment           = False
        rf.valid_dtcs_codes      = DTCS_CODES
        rf.valid_name_length     = 10
        rf.valid_power_levels    = UVK5_POWER_LEVELS
        rf.valid_special_chans   = []
        rf.valid_duplexes        = ["", "-", "+", "off"]
        rf.valid_tuning_steps    = STEPS
        rf.valid_tmodes          = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes     = ["Tone->Tone", "Tone->DTCS", "DTCS->Tone",
                                    "->Tone", "->DTCS", "DTCS->", "DTCS->DTCS"]
        rf.valid_characters      = chirp_common.CHARSET_ASCII
        rf.valid_modes           = ["FM", "AM", "NAM", "USB", "CW", "WFM" , "DIG"] #ordine con cui chirp le visualizza nelle mmeorie...allineato a quelli della radio 
        rf.valid_skips           = ["", "S"]
        rf._expanded_limits      = True
        rf.memory_bounds         = (1, CHAN_MAX)   # 500 canali
        rf.valid_bands = []
        for a in BANDS:
            rf.valid_bands.append(
                (int(BANDS[a][0]*1000000),
                 int(BANDS[a][1]*1000000)))
        return rf

#--------------------------------------------------------------------------------
    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

#--------------------------------------------------------------------------------
    def sync_out(self):
        do_upload(self)

#--------------------------------------------------------------------------------
    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)
        self._group_list = build_group_list(self._memobj)
        bank_index = get_bank_index(self._memobj, self.ACTIVE_BANK)
        if bank_index is not None:
            self.ACTIVE_BANK = bank_index
        self.ACTIVE_BANK_NAME = get_bank_name(self._memobj, self.ACTIVE_BANK)

#--------------------------------------------------------------------------------
    def _get_group_list(self):
        group_list = getattr(self, "_group_list", None)
        if not group_list or len(group_list) != 16:
            group_list = build_group_list(self._memobj)
            self._group_list = group_list
        return list(group_list)

#--------------------------------------------------------------------------------
    def _get_bank_label(self):
        bank_index = self.ACTIVE_BANK
        if bank_index is None:
            bank_index = get_bank_index(self._memobj, None)

        bank_name = (getattr(self, "ACTIVE_BANK_NAME", "") or "").strip()
        if not bank_name:
            bank_name = get_bank_name(self._memobj, bank_index)

        if bank_index is None:
            return bank_name or "Bank unknown"
        if bank_name:
            return "Bank %02d - %s" % (bank_index, bank_name)
        return "Bank %02d" % bank_index

#--------------------------------------------------------------------------------
    def _append_bank_info(self, extra_group):
        bank_label = self._get_bank_label()
        if not bank_label:
            return

        val = RadioSettingValueString(len(bank_label), len(bank_label), bank_label)
        val.set_mutable(False)
        extra_group.append(RadioSetting("bank_info", "Bank", val))

#--------------------------------------------------------------------------------
    def _group_index_from_value(self, value, current_index=0):
        group_list = self._get_group_list()
        text = str(value).strip()
        if text in group_list:
            return group_list.index(text)

        upper = text.upper()
        if upper == "ALL":
            return 0
        if upper.startswith("GROUP "):
            suffix = upper[6:].strip()
            if suffix.isdigit():
                index = int(suffix)
                if 0 <= index < len(group_list):
                    return index

        return current_index if 0 <= current_index < len(group_list) else 0

#--------------------------------------------------------------------------------
    def get_raw_memory(self, number):
        if not 1 <= number <= CHAN_MAX:
            return ""
        return repr(self._memobj.channel[number-1])

#-------------------------------------------------------------------------------- VALIDAZIONE FREQUENZA
    def validate_memory(self, mem):
        msgs = super().validate_memory(mem)
        if mem.duplex == 'off':
            return msgs
        if mem.duplex == '-':
            txfreq = mem.freq - mem.offset
        elif mem.duplex == '+':
            txfreq = mem.freq + mem.offset
        else:
            txfreq = mem.freq
        band = _find_band(txfreq)
        if band is False:
            msg = "Transmit frequency %.4f MHz is not supported by this radio" % (txfreq/1000000.0)
            msgs.append(chirp_common.ValidationError(msg))
        band = _find_band(mem.freq)
        if band is False:
            msg = "The frequency %.4f MHz is not supported by this radio" % (mem.freq/1000000.0)
            msgs.append(chirp_common.ValidationError(msg))
        return msgs

#-------------------------------------------------------------------------------- IMPOSTA TONI
    def _set_tone(self, mem, _mem):
        ((txmode, txtone, txpol),
         (rxmode, rxtone, rxpol)) = chirp_common.split_tone_encode(mem)

        if txmode == "Tone":
            txtoval = CTCSS_TONES.index(txtone)
            txmoval = 0b01
        elif txmode == "DTCS":
            txmoval = txpol == "R" and 0b11 or 0b10
            txtoval = DTCS_CODES.index(txtone)
        else:
            txmoval = 0
            txtoval = 0

        if rxmode == "Tone":
            rxtoval = CTCSS_TONES.index(rxtone)
            rxmoval = 0b01
        elif rxmode == "DTCS":
            rxmoval = rxpol == "R" and 0b11 or 0b10
            rxtoval = DTCS_CODES.index(rxtone)
        else:
            rxmoval = 0
            rxtoval = 0

        _mem.rx_codetype = rxmoval
        _mem.tx_codetype = txmoval
        _mem.rxcode      = rxtoval
        _mem.txcode      = txtoval

#-------------------------------------------------------------------------------- LEGGI TONI
    def _get_tone(self, mem, _mem):
        rxtype  = _mem.rx_codetype
        txtype  = _mem.tx_codetype
        rx_tmode = TMODES[rxtype]
        tx_tmode = TMODES[txtype]
        rx_tone = tx_tone = None

        if tx_tmode == "Tone":
            if _mem.txcode < len(CTCSS_TONES):
                tx_tone = CTCSS_TONES[_mem.txcode]
            else:
                tx_tone  = 0
                tx_tmode = ""
        elif tx_tmode == "DTCS":
            if _mem.txcode < len(DTCS_CODES):
                tx_tone = DTCS_CODES[_mem.txcode]
            else:
                tx_tone  = 0
                tx_tmode = ""

        if rx_tmode == "Tone":
            if _mem.rxcode < len(CTCSS_TONES):
                rx_tone = CTCSS_TONES[_mem.rxcode]
            else:
                rx_tone  = 0
                rx_tmode = ""
        elif rx_tmode == "DTCS":
            if _mem.rxcode < len(DTCS_CODES):
                rx_tone = DTCS_CODES[_mem.rxcode]
            else:
                rx_tone  = 0
                rx_tmode = ""

        tx_pol = txtype == 0x03 and "R" or "N"
        rx_pol = rxtype == 0x03 and "R" or "N"
        chirp_common.split_tone_decode(mem, (tx_tmode, tx_tone, tx_pol), (rx_tmode, rx_tone, rx_pol))

################################################################################################################################
#                                                                                                 L E T T U R A   M E M O R I E
################################################################################################################################

#--------------------------------------------------------------------------------
    def get_memory(self, number2):
        group_list = self._get_group_list()

        mem = chirp_common.Memory()

        if isinstance(number2, str):
            number = SPECIALS[number2]
            mem.extd_number = number2
        else:
            number = number2 - 1

        if number < 0 or number >= CHAN_MAX:
            mem.number = number + 1
            mem.empty = True
            mem.immutable = ["name", "scanlists", "freq", "offset", "duplex", "mode", "tmode", "rtone", "ctone", "dtcs", "rx_dtcs", "cross_mode", "power"]
            return mem

        mem.number = number + 1
        _mem = self._memobj.channel[number]
        tmpcomment = ""

        # canale vuoto?
        is_empty = False
        if (_mem.freq == 0xffffffff) or (_mem.freq == 0) or (_mem.band == 0xF):
            is_empty = True

        if is_empty:
            mem.empty = True
            mem.power = UVK5_POWER_LEVELS[2]
            mem.extra = RadioSettingGroup("Extra", "extra")
            self._append_bank_info(mem.extra)
            rs = RadioSetting("bandwidth", "Bandwidth", RadioSettingValueList(BANDWIDTH_LIST, BANDWIDTH_LIST[0]))
            mem.extra.append(rs)
            rs = RadioSetting("frev", "FreqRev", RadioSettingValueBoolean(False))
            mem.extra.append(rs)
            rs = RadioSetting("pttid", "PTTID", RadioSettingValueList(PTTID_LIST, PTTID_LIST[0]))
            mem.extra.append(rs)
            rs = RadioSetting("agcmode", _("AGC mode"), RadioSettingValueList(AGC_MODE, AGC_MODE[0]))
            mem.extra.append(rs)
            rs = RadioSetting("compander", _("Compander"), RadioSettingValueList(COMPANDER_LIST, COMPANDER_LIST[0]))
            mem.extra.append(rs)
            rs = RadioSetting("scrambler", _("Scrambler"), RadioSettingValueBoolean(False))
            mem.extra.append(rs)
            rs = RadioSetting("squelch", _("Squelch"), RadioSettingValueList(SQUELCH_LIST, SQUELCH_LIST[1]))
            mem.extra.append(rs)
            rs = RadioSetting("writeprot", _("Write Protect"), RadioSettingValueBoolean(False))
            mem.extra.append(rs)
            rs = RadioSetting("txlock", _("TX Lock"), RadioSettingValueBoolean(False))
            mem.extra.append(rs)
            rs = RadioSetting("group", "Group", RadioSettingValueList(group_list, group_list[0]))
            mem.extra.append(rs)
            rs = RadioSetting("busylock", "Busy Lock", RadioSettingValueBoolean(False))
            mem.extra.append(rs)
            return mem

        if number > (CHAN_MAX-1):
            mem.immutable = ["name", "scanlists"]
        else:
            _mem2 = self._memobj.channel[number]
            for char in _mem2.name:
                if str(char) == "\xFF" or str(char) == "\x00":
                    break
                mem.name += str(char)
            mem.name = mem.name.strip()

        raw = _mem.get_raw(asbytes=True)
        b3 = raw[0x18 + 3] if raw and len(raw) > (0x18 + 3) else 0
        raw_shift = b3 & 0x03
        raw_mod = (b3 >> 2) & 0x07
        raw_noscan = (b3 >> 5) & 0x01
        raw_writeprot = (b3 >> 6) & 0x01
        raw_txlock = (b3 >> 7) & 0x01

        # frequenza e offset
        mem.freq   = int(_mem.freq) * 10
        mem.offset = int(_mem.offset) * 10

        if mem.offset == 0:
            mem.duplex = ''
        else:
            if raw_shift == OFFSET_MINUS:
                if _mem.freq == _mem.offset:
                    mem.duplex = 'off'
                    mem.offset = 0
                else:
                    mem.duplex = '-'
            elif raw_shift == OFFSET_PLUS:
                mem.duplex = '+'
            else:
                mem.duplex = ''

        # toni
        self._get_tone(mem, _mem)

        # modulazione
        if raw_mod < len(MODULATION_LIST):
            mem.mode = MODULATION_LIST[raw_mod]
        else:
            mem.mode = "FM"

        # step
        raw_step = int(_mem.step_squelch)
        tstep = raw_step & 0x0F
        if tstep < len(STEPS):
            mem.tuning_step = STEPS[tstep]
        else:
            mem.tuning_step = 0.02

        # scan skip
        if raw_noscan < len(SKIP_VALUES):
            mem.skip = SKIP_VALUES[raw_noscan]
        else:
            mem.skip = ""

        # potenza TX
        if _mem.txpower == POWER_HIGH:
            mem.power = UVK5_POWER_LEVELS[2]
        elif _mem.txpower == POWER_MEDIUM:
            mem.power = UVK5_POWER_LEVELS[1]
        else:
            mem.power = UVK5_POWER_LEVELS[0]

        if (_mem.freq == 0xffffffff) or (_mem.freq == 0):
            mem.empty = True
        else:
            mem.empty = False

        mem.extra = RadioSettingGroup("Extra", "extra")
        self._append_bank_info(mem.extra)

        # Bandwidth
        raw = _mem.get_raw(asbytes=True)
        b4 = raw[0x18 + 4] if raw and len(raw) > (0x18 + 4) else 0
        bwidth = (b4 >> 1) & 0x0F
        if bwidth >= len(BANDWIDTH_LIST):
            bwidth = 0
        rs = RadioSetting("bandwidth", "Bandwidth", RadioSettingValueList(BANDWIDTH_LIST, BANDWIDTH_LIST[bwidth]))
        mem.extra.append(rs)
        tmpcomment += "bandwidth:" + BANDWIDTH_LIST[bwidth] + " "

        # Gruppo
        group = _mem.group
        if group >= len(group_list):
            group = 0
        rs = RadioSetting("group", "Group", RadioSettingValueList(group_list, group_list[group]))
        mem.extra.append(rs)
        tmpcomment += group_list[group] + " "

        # Freq reverse
        is_frev = bool(_mem.reverse > 0)
        rs = RadioSetting("frev", "FreqRev", RadioSettingValueList(["OFF", "ON"], "ON" if is_frev else "OFF"))
        mem.extra.append(rs)
        tmpcomment += "FreqReverse:" + (is_frev and "ON" or "OFF") + " "

        # PTTID
        raw_dig = int(_mem.dig_ptt)
        pttid = (raw_dig >> 4) & 0x0F
        if pttid >= len(PTTID_LIST):
            pttid = 0
        rs = RadioSetting("pttid", "PTTID", RadioSettingValueList(PTTID_LIST, PTTID_LIST[pttid]))
        mem.extra.append(rs)
        tmpcomment += "PTTid:" + PTTID_LIST[pttid] + " "

        # Codici selettive
        codesel = hexasc(_mem.code_sel0) + hexasc(_mem.code_sel1) + \
                  hexasc(_mem.code_sel2) + hexasc(_mem.code_sel3) + \
                  hexasc(_mem.code_sel4) + hexasc(_mem.code_sel5) + \
                  hexasc(_mem.code_sel6) + hexasc(_mem.code_sel7) + \
                  hexasc(_mem.code_sel8) + hexasc(_mem.code_sel9)
        rs = RadioSetting("codesel", "Own ID", RadioSettingValueString(0, 10, codesel))
        mem.extra.append(rs)
        tmpcomment += "PTTid Codes:" + codesel + " "

        # Digital Code
        raw_dig = int(_mem.dig_ptt)
        enc = raw_dig & 0x0F
        if enc >= len(DIGITAL_CODE_LIST):
            enc = 0
        rs = RadioSetting("DIGCode", _("DIGCode"), RadioSettingValueList(DIGITAL_CODE_LIST, DIGITAL_CODE_LIST[enc]))
        mem.extra.append(rs)
        tmpcomment += "DIGCode:" + DIGITAL_CODE_LIST[enc] + " "

        # AGC mode
        enc = _mem.agcmode if _mem.agcmode < len(AGC_MODE) else 0
        rs = RadioSetting("agcmode", _("AGC mode"), RadioSettingValueList(AGC_MODE, AGC_MODE[enc]))
        mem.extra.append(rs)
        tmpcomment += "AGC Mode:" + AGC_MODE[enc] + " "

        # Compander
        comp = _mem.compander
        if comp >= len(COMPANDER_LIST):
            comp = 0
        rs = RadioSetting("compander", _("Compander"), RadioSettingValueList(COMPANDER_LIST, COMPANDER_LIST[comp]))
        mem.extra.append(rs)
        tmpcomment += "Compander:" + COMPANDER_LIST[comp] + " "

        # Scrambler (bool per canali)
        scr = bool(_mem.scrambler > 0)
        rs = RadioSetting("scrambler", _("Scrambler"), RadioSettingValueList(["OFF", "ON"], "ON" if scr else "OFF"))
        mem.extra.append(rs)
        tmpcomment += "Scrambler:" + (scr and "ON" or "OFF") + " "

        # Squelch
        raw_step = int(_mem.step_squelch)
        sql = (raw_step >> 4) & 0x0F
        if sql >= len(SQUELCH_LIST):
            sql = 1
        rs = RadioSetting("squelch", _("Squelch"), RadioSettingValueList(SQUELCH_LIST, SQUELCH_LIST[sql]))
        mem.extra.append(rs)
        tmpcomment += SQUELCH_LIST[sql] + " "

        # Busy Lock
        bl = bool(_mem.busylock > 0)
        rs = RadioSetting("busylock", "Busy Lock", RadioSettingValueList(["OFF", "ON"], "ON" if bl else "OFF"))
        mem.extra.append(rs)
        tmpcomment += "Busy Lock:" + (bl and "ON" or "OFF") + " "

        # Write Protect
        wp = bool(raw_writeprot > 0)
        rs = RadioSetting("writeprot", _("Write Protect"), RadioSettingValueList(["OFF", "ON"], "ON" if wp else "OFF"))
        mem.extra.append(rs)
        tmpcomment += "Write Protect:" + (wp and "ON" or "OFF") + " "

        # TX Lock
        wp = bool(raw_txlock > 0)
        rs = RadioSetting("txlock", _("TX Lock"), RadioSettingValueList(["OFF", "ON"], "ON" if wp else "OFF"))
        mem.extra.append(rs)
        tmpcomment += "TX Lock:" + (wp and "ON" or "OFF") + " "

        return mem

################################################################################################################################
#                                                                                         S A L V A T A G G I O   M E M O R I E
################################################################################################################################

#--------------------------------------------------------------------------------
    def set_memory(self, mem):
        number = mem.number - 1
        group_list = self._get_group_list()

        if number < 0 or number >= CHAN_MAX:
            return mem

        _mem = self._memobj.channel[number]

        # canale prima vuoto?
        if _mem.get_raw(asbytes=False)[0] == "\xff":
            _mem.set_raw("\x00" * 32)
            for i in range(10):
                setattr(_mem, "code_sel%d" % i if i < 10 else "code_sel%s" % chr(55+i), 14)
            _mem.code_sel0 = _mem.code_sel1 = _mem.code_sel2 = _mem.code_sel3 = 14
            _mem.code_sel4 = _mem.code_sel5 = _mem.code_sel6 = _mem.code_sel7 = 14
            _mem.code_sel8 = _mem.code_sel9 = 14

        # banda
        _mem.band = _find_band(mem.freq)

        # modulazione
        if mem.mode in MODULATION_LIST:
            modulation = MODULATION_LIST.index(mem.mode)
        else:
            modulation = 0

        # frequenza / offset
        _mem.freq   = mem.freq / 10
        _mem.offset = mem.offset / 10

        if mem.duplex == "":
            _mem.offset = 0
            shift = 0
        elif mem.duplex == '-':
            shift = OFFSET_MINUS
        elif mem.duplex == '+':
            shift = OFFSET_PLUS
        elif mem.duplex == 'off':
            shift = OFFSET_MINUS
            _mem.offset = _mem.freq
        else:
            shift = 0

        # nome
        _mem.name = mem.name.ljust(10)

        # toni
        self._set_tone(mem, _mem)

        # step
        if mem.tuning_step in STEPS:
            val = STEPS.index(mem.tuning_step) & 0x0F
        else:
            val = 0
        cur = int(_mem.step_squelch) & 0xF0
        _mem.step_squelch = cur | val

        # potenza TX
        if str(mem.power) == str(UVK5_POWER_LEVELS[2]):
            _mem.txpower = POWER_HIGH
        elif str(mem.power) == str(UVK5_POWER_LEVELS[1]):
            _mem.txpower = POWER_MEDIUM
        else:
            _mem.txpower = POWER_LOW

        # scan skip
        if mem.skip in SKIP_VALUES:
            enablescan = SKIP_VALUES.index(mem.skip)
        else:
            enablescan = 0

        # extra
        bandwidth = int(_mem.bw) & 0x0F
        reverse = int(_mem.reverse) & 0x01
        busylock = int(_mem.busylock) & 0x01
        writeprot = 0
        txlock = 0
        for setting in mem.extra:
            sname  = setting.get_name()
            svalue = setting.value.get_value()

            if sname == "bandwidth":
                bandwidth = BANDWIDTH_LIST.index(svalue) & 0x0F
            if sname == "pttid":
                cur = int(_mem.dig_ptt) & 0x0F
                val = (PTTID_LIST.index(svalue) & 0x0F) << 4
                _mem.dig_ptt = cur | val
            if sname == "frev":
                reverse = (str(svalue).upper() == "ON") and 1 or 0
            if sname == "DIGCode":
                cur = int(_mem.dig_ptt) & 0xF0
                val = DIGITAL_CODE_LIST.index(svalue) & 0x0F
                _mem.dig_ptt = cur | val
            if sname == "agcmode":
                _mem.agcmode = AGC_MODE.index(svalue)
            if sname == "compander":
                _mem.compander = COMPANDER_LIST.index(svalue)
            if sname == "scrambler":
                _mem.scrambler = (str(svalue).upper() == "ON") and 1 or 0
            if sname == "group":
                current_group = int(_mem.group) if int(_mem.group) < len(group_list) else 0
                _mem.group = self._group_index_from_value(svalue, current_group)
            if sname == "squelch":
                cur = int(_mem.step_squelch) & 0x0F
                val = (SQUELCH_LIST.index(svalue) & 0x0F) << 4
                _mem.step_squelch = cur | val
            if sname == "busylock":
                busylock = (str(svalue).upper() == "ON") and 1 or 0
            if sname == "writeprot":
                writeprot = (str(svalue).upper() == "ON") and 1 or 0
            if sname == "txlock":
                txlock = (str(svalue).upper() == "ON") and 1 or 0
            if sname == "codesel":
                _mem.code_sel0 = ascdec(svalue[0])
                _mem.code_sel1 = ascdec(svalue[1])
                _mem.code_sel2 = ascdec(svalue[2])
                _mem.code_sel3 = ascdec(svalue[3])
                _mem.code_sel4 = ascdec(svalue[4])
                _mem.code_sel5 = ascdec(svalue[5])
                _mem.code_sel6 = ascdec(svalue[6])
                _mem.code_sel7 = ascdec(svalue[7])
                _mem.code_sel8 = ascdec(svalue[8])
                _mem.code_sel9 = ascdec(svalue[9])

        # aggiorna byte3/byte4 con bitmask
        raw = bytearray(_mem.get_raw(asbytes=True))
        if len(raw) >= 32:
            b3 = (shift & 0x03) | ((modulation & 0x07) << 2) | ((enablescan & 0x01) << 5) | ((writeprot & 0x01) << 6) | ((txlock & 0x01) << 7)
            raw[0x18 + 3] = b3
            b4 = (reverse & 0x01) | ((bandwidth & 0x0F) << 1) | ((_mem.txpower & 0x03) << 5) | ((busylock & 0x01) << 7)
            raw[0x18 + 4] = b4
            _mem.set_raw(bytes(raw))

        # canale rimasto vuoto (freq=0)?
        if _mem.freq == 0:
            _mem.set_raw("\xFF" * 32)
            _mem.code_sel0 = _mem.code_sel1 = _mem.code_sel2 = _mem.code_sel3 = 14
            _mem.code_sel4 = _mem.code_sel5 = _mem.code_sel6 = _mem.code_sel7 = 14
            _mem.code_sel8 = _mem.code_sel9 = 14

        return mem

################################################################################################################################
#                                                                                      B A N K   /   G R O U P   S E T T I N G S
################################################################################################################################

#--------------------------------------------------------------------------------
    def get_settings(self):
        group_list = self._get_group_list()
        bank_group = RadioSettingGroup("bank_groups", "Bank / Groups")

        bank_label = self._get_bank_label()
        val = RadioSettingValueString(len(bank_label), len(bank_label), bank_label)
        val.set_mutable(False)
        bank_group.append(RadioSetting("bank_info", "Active bank", val))

        note = "To edit another bank, select it on the radio and download again."
        val = RadioSettingValueString(len(note), len(note), note)
        val.set_mutable(False)
        bank_group.append(RadioSetting("bank_note", "Note", val))

        bank_name = get_bank_name(self._memobj, self.ACTIVE_BANK)
        val = RadioSettingValueString(len(bank_name), len(bank_name), bank_name)
        val.set_mutable(False)
        bank_group.append(
            RadioSetting(
                "bank_name",
                "Bank name",
                val,
            )
        )

        for index in range(16):
            bank_group.append(
                RadioSetting(
                    "group_name_%d" % index,
                    "Group %d" % index,
                    RadioSettingValueString(0, 8, group_list[index]),
                )
            )

        return RadioSettings(bank_group)

#--------------------------------------------------------------------------------
    def set_settings(self, settings):
        _mem = self._memobj

        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue

            name = element.get_name()

            if name.startswith("group_name_"):
                try:
                    index = int(name.rsplit("_", 1)[1])
                except Exception:
                    LOG.exception("Invalid group setting name: %s", name)
                    continue

                if 0 <= index < 16:
                    _mem.list_name[index].name = _sanitize_ascii_name(
                        str(element.value),
                        GROUP_FALLBACK_LIST[index],
                        8,
                    )

        self._group_list = build_group_list(_mem)
        self.ACTIVE_BANK = get_bank_index(_mem, self.ACTIVE_BANK)
        self.ACTIVE_BANK_NAME = get_bank_name(_mem, self.ACTIVE_BANK)
        if self.ACTIVE_BANK is not None:
            _mem.mem_bank = self.ACTIVE_BANK
