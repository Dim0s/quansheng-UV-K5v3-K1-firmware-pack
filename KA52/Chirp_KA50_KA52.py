# CHIRP driver for Quansheng KA-50
# Byte layout identical to Sonic/UV-K5 V3 — compatible with existing EEPROM
# KA-50 additions:
#   - Modulation 3 = CW (in addition to FM/AM/DSB)
#   - Extended bandwidth: byte[0x0D] bit0 = bw_ext (for DSB/CW sub-bands)
#   - FM radio: 2 banks x 6 channels (0xA070/0xA078/0xA080/0xA088)
#   - Side keys: no A/B, RXMODE, MAINONLY; has PTT, WN, MUTE, MODE

import os
import struct
import logging

from chirp import chirp_common, directory, bitwise, memmap, errors, util
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueBoolean, RadioSettingValueList, \
    RadioSettingValueInteger, RadioSettingValueString, \
    RadioSettings

LOG = logging.getLogger(__name__)

DRIVER_VERSION = "Quansheng KA-50 CHIRP driver 2026/06/14"  # fixed: bw_ext, validate_memory, duplex=off TX ban

MEM_FORMAT = """
#seekto 0x000000;
struct {
  ul32 freq;
  ul32 offset;
  u8 rxcode;
  u8 txcode;
  u8 txcodeflag:4, rxcodeflag:4;
  u8 modulation:4, offsetDir:4;

  u8 byte0C;  // decoded manually: new(bit7=1)=bits[6:4]=pwr,bits[3:1]=bw | old(bit7=0)=bits[4:2]=pwr,bit1=bw

  u8 byte0D;  // bit0=bw_ext(CW), bit1=tx_ban(KA50), bits[3:2]=dtmf(Sonic compat)

  u8 step;
  u8 __u0F;
} channel[1024];

#seekto 0x004000;
struct { char name[16]; } channelname[1024];

#seekto 0x008000;
struct {
  u8 __u_attr:3, compander:2, band:3;
  u8 scanlist;
} ch_attr[1031];

#seekto 0x009000;
struct {
  ul32 freq;
  ul32 offset;
  u8 rxcode;
  u8 txcode;
  u8 txcodeflag:4, rxcodeflag:4;
  u8 modulation:4, offsetDir:4;
  u8 byte0C;
  u8 byte0D;
  u8 step;
  u8 __uVFO2;
} vfo_channel[14];

#seekto 0x00A000;
u8 set_rxa;
u8 squelch;
u8 max_talk_time;
u8 noaa_autoscan;
u8 __u4:1, set_nav:1, set_key:4, set_menu_lock:1, key_lock:1;
u8 vox_switch;
u8 vox_level;
u8 mic_gain;

#seekto 0x00A008;
u8 backlight_min:4, backlight_max:4;
u8 channel_display_mode;
u8 crossband;
u8 battery_save;
u8 dual_watch;
u8 backlight_time;
u8 __u5:5, set_nfm:2, ste:1;
u8 current_state;

#seekto 0x00A010;
ul16 ScreenChannel_A; ul16 MrChannel_A; ul16 FreqChannel_A;
ul16 ScreenChannel_B; ul16 MrChannel_B; ul16 FreqChannel_B;
ul16 NoaaChannel_A;   ul16 NoaaChannel_B;

// FM 2 banks x 6 channels
#seekto 0x00A070;
ul16 fm_bank1[4];
#seekto 0x00A078;
ul16 fm_bank1_45[2];
u8   fm_display_style;
u8   fm_active_bank;
#seekto 0x00A080;
ul16 fm_bank2[4];
#seekto 0x00A088;
ul16 fm_bank2_45[2];

#seekto 0x00A0A8;
u8 keyM_longpress_action:7, button_beep:1;
u8 key1_shortpress_action;
u8 key1_longpress_action;
u8 key2_shortpress_action;
u8 key2_longpress_action;
u8 scan_resume_mode;
u8 auto_keypad_lock;
u8 power_on_dispmode;
ul32 password;

#seekto 0x00A0B8;
u8 voice;
i8 dbm_corr[7];

#seekto 0x00A0C0;
u8 alarm_mode; u8 roger_beep; u8 rp_ste; u8 TX_VFO; u8 Battery_type;

#seekto 0x00A0C8;
char logo_line1[16];
char logo_line2[16];

#seekto 0x00A130;
struct {
  u8 slPriorEnab:1, slDef:7;
  ul16 slPriorCh1; ul16 slPriorCh2; ul16 call_channel;
  u8 __u6;
} sl;

#seekto 0x00A150;
u8 int_flock; u8 int_350tx; u8 int_KILLED; u8 int_200tx;
u8 int_500tx; u8 int_350en; u8 int_scren;
u8 backlight_on_TX_RX:2, AM_fix:1, mic_bar:1,
   battery_text:2, live_DTMF_decoder:1, __u7:1;

#seekto 0x00A158;
struct {
  u8 ENABLE_DTMF_CALLING:1, ENABLE_PWRON_PASSWORD:1,
     ENABLE_TX1750:1, ENABLE_ALARM:1, ENABLE_VOX:1,
     ENABLE_VOICE:1, ENABLE_NOAA:1, ENABLE_FMRADIO:1;
  u8 __u8:1, ENABLE_FEAT_F4HWN_RESCUE_OPS:1,
     ENABLE_BANDSCOPE:1, ENABLE_AM_FIX:1,
     ENABLE_FEAT_F4HWN_GAME:1, ENABLE_RAW_DEMODULATORS:1,
     ENABLE_WIDE_RX:1, ENABLE_FLASHLIGHT:1;
} BUILD_OPTIONS;

u8 __u9; u8 __uA;
u8 set_off_tmr:7, set_tmr:1;
u8 set_gui:1, set_met:1, set_lck:1, set_inv:1, set_contrast:4;
u8 set_tot:4, set_eot:4;
u8 set_pwr:4, set_ptt:4;

#seekto 0x00A160;
struct { char version[16]; } version;

struct {
  struct {
    #seekto 0x00B000;
    u8 openRssiThr[10];
    #seekto 0x00B010;
    u8 closeRssiThr[10];
    #seekto 0x00B020;
    u8 openNoiseThr[10];
    #seekto 0x00B030;
    u8 closeNoiseThr[10];
    #seekto 0x00B040;
    u8 closeGlitchThr[10];
    #seekto 0x00B050;
    u8 openGlitchThr[10];
  } sqlBand4_7;
  struct {
    #seekto 0x00B060;
    u8 openRssiThr[10];
    #seekto 0x00B070;
    u8 closeRssiThr[10];
    #seekto 0x00B080;
    u8 openNoiseThr[10];
    #seekto 0x00B090;
    u8 closeNoiseThr[10];
    #seekto 0x00B0A0;
    u8 closeGlitchThr[10];
    #seekto 0x00B0B0;
    u8 openGlitchThr[10];
  } sqlBand1_3;
  #seekto 0x00B0C0;
  struct { ul16 level1; ul16 level2; ul16 level4; ul16 level6; } rssiLevelsBands3_7;
  struct { ul16 level1; ul16 level2; ul16 level4; ul16 level6; } rssiLevelsBands1_2;
  struct {
    struct { u8 lower; u8 center; u8 upper; } low;
    struct { u8 lower; u8 center; u8 upper; } mid;
    struct { u8 lower; u8 center; u8 upper; } hi;
    #seek 7;
  } txp[7];
  #seekto 0x00B140;
  ul16 batLvl[6];
  #seekto 0x00B150;
  ul16 vox1Thr[10];
  #seekto 0x00B168;
  ul16 vox0Thr[10];
  #seekto 0x00B180;
  u8 micLevel[5];
  #seekto 0x00B188;
  il16 xtalFreqLow;
  #seekto 0x00B18E;
  u8 volumeGain;
  u8 dacGain;
} cal;
"""

# ── Constants ────────────────────────────────────────────────────────────────
MR_CHANNELS_MAX = 1024
MEM_SIZE        = 0x00B190
PROG_SIZE       = 0x00A171
MEM_BLOCK       = 0x40  # VCP_RX_BUF_SIZE=128, пакет=20+dlen, max dlen=64
FMMIN = 76.0; FMMAX = 108.0

FLAGS1_OFFSET_NONE  = 0b00
FLAGS1_OFFSET_MINUS = 0b10
FLAGS1_OFFSET_PLUS  = 0b01
# KA-50 OUTPUT_POWER (settings.h): 0=X(TX off), 1=LOW, 2=MID, 3=HIGH, 4=USER
OUTPUT_POWER_X    = 0
OUTPUT_POWER_LOW  = 1
OUTPUT_POWER_MID  = 2
OUTPUT_POWER_HIGH = 3
OUTPUT_POWER_USER = 4

# KA-50 modulation enum: 0=FM 1=AM 2=DSB(→USB in CHIRP) 3=CW
VALID_MODES = ["FM", "NFM", "AM", "NAM", "USB", "CW"]

# KA-50 ACTION_OPT values
KEYACTIONS_LIST = [
    "NONE",          # 0
    "FLASHLIGHT",    # 1
    "POWER",         # 2
    "MONITOR",       # 3
    "SCAN",          # 4
    "",              # 5 VOX — нет в меню
    "",              # 6 ALARM
    "FM RADIO",      # 7
    "",              # 8 1750Hz
    "LOCK KEYPAD",   # 9
    "",              # 10 A/B — удалён
    "VFO / MEM",     # 11
    "MODE",          # 12
    "",              # 13 BLMIN
    "PTT",           # 14
    "WIDE / NARROW", # 15
    "",              # 16 BACKLIGHT — нет в меню
    "MUTE",          # 17
]
KEYACTIONS_VISIBLE = [k for k in KEYACTIONS_LIST if k]

UVK5_POWER_LEVELS = [chirp_common.PowerLevel("LOW"),
                     chirp_common.PowerLevel("MID"),
                     chirp_common.PowerLevel("HIGH"),
                     chirp_common.PowerLevel("USER")]

COMPANDER_LIST   = ["OFF", "TX", "RX", "TX/RX"]
CHANNELDISP_LIST = ["Frequency", "Channel Number", "Name", "Name+Freq"]
BATSAVE_LIST     = ["OFF", "1:1", "1:2", "1:3", "1:4", "1:5"]
BATTYPE_LIST     = ["1600mAh", "2200mAh", "3500mAh", "1400mAh K1", "2500mAh K1"]
BAT_TXT_LIST     = ["NONE", "VOLTAGE", "PERCENT"]
BL_TX_RX_LIST    = ["OFF", "TX", "RX", "TX/RX"]
BACKLIGHT_LIST   = ["OFF","5s","10s","20s","30s","1min","2min","3min","Always"]
BL_LVL_LIST      = [str(i) for i in range(11)]
ROGER_LIST       = ["OFF","MARIO","BLAST","R2D2","ROGER","AMBUL","OURO","KLAC","PIU","ICQ"]
SCANRESUME_LIST  = ["STOP", "CARRIER", "TIMEOUT"]
WELCOME_LIST     = ["ALL", "SOUND", "MESSAGE", "VOLTAGE", "NONE"]
MIC_GAIN_LIST    = ["+1.5dB","+4.0dB","+8.0dB","+12.0dB",
                    "+16.0dB","+20.0dB","+24.0dB","+28.0dB","+31.5dB"]
FLOCK_LIST       = ["PMR 446","136-500 MHz","UNLOCK ALL","DISABLE ALL"]
ALARMMODE_LIST   = ["SITE", "TONE"]
TX_VFO_LIST      = ["A", "B"]
SCANLIST_LIST    = ["None"] + [f"List {i}" for i in range(1, 33)]

STEPS = [0.01,0.05,0.1,0.25,0.5,1,1.25,2.5,5,6.25,8.33,9,10,
         12.5,15,20,25,30,50,100,125,200,250,500]
TMODES = ["", "Tone", "DTCS", "DTCS"]

CTCSS_TONES = [
    67.0,69.3,71.9,74.4,77.0,79.7,82.5,85.4,88.5,91.5,94.8,97.4,
    100.0,103.5,107.2,110.9,114.8,118.8,123.0,127.3,131.8,136.5,
    141.3,146.2,151.4,156.7,159.8,162.2,165.5,167.9,171.3,173.8,
    177.3,179.9,183.5,186.2,189.9,192.8,196.6,199.5,203.5,206.5,
    210.7,218.1,225.7,229.1,233.6,241.8,250.3,254.1
]
DTCS_CODES = [
    23,25,26,31,32,36,43,47,51,53,54,65,71,72,73,74,114,115,116,
    122,125,131,132,134,143,145,152,155,156,162,165,172,174,205,
    212,223,225,226,243,244,245,246,251,252,255,261,263,265,266,
    271,274,306,311,315,325,331,332,343,346,351,356,364,365,371,
    411,412,413,423,431,432,445,446,452,454,455,462,464,465,466,
    503,506,516,523,526,532,546,565,606,612,624,627,631,632,654,
    662,664,703,712,723,731,732,734,743,754
]

BANDS_STANDARD = {0:[50.0,76.0],1:[108.0,136.9999],2:[137.0,173.9999],
                  3:[174.0,349.9999],4:[350.0,399.9999],5:[400.0,469.9999],6:[470.0,600.0]}
BANDS_WIDE     = {0:[18.0,108.0],1:[108.0,136.9999],2:[137.0,173.9999],
                  3:[174.0,349.9999],4:[350.0,399.9999],5:[400.0,469.9999],6:[470.0,1300.0]}

# ── Protocol (same as Sonic) ─────────────────────────────────────────────────
def xorarr(data):
    tbl = [22,108,20,230,46,145,13,64,33,53,213,64,19,3,233,128]
    ret = b""; idx = 0
    for b in data:
        ret += bytes([b ^ tbl[idx]]); idx = (idx+1) % len(tbl)
    return ret

def calculate_crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc

def _send_command(serport, data):
    crc   = calculate_crc16_xmodem(data)
    data2 = data + struct.pack("<H", crc)
    cmd   = struct.pack(">HBB", 0xABCD, len(data), 0) + xorarr(data2) + struct.pack(">H", 0xDCBA)
    try:
        serport.write(cmd)
        serport.flush()
    except Exception as e:
        raise errors.RadioError("Error writing to radio") from e


def _receive_reply(serport):
    header = serport.read(4)
    if len(header) != 4:
        raise errors.RadioError("Header short read")
    if header[0] != 0xAB or header[1] != 0xCD or header[3] != 0x00:
        raise errors.RadioError("Bad response header")
    cmd = serport.read(int(header[2]))
    if len(cmd) != int(header[2]):
        raise errors.RadioError("Body short read")
    footer = serport.read(4)
    if len(footer) != 4 or footer[2] != 0xDC or footer[3] != 0xBA:
        raise errors.RadioError("Bad response footer")
    return xorarr(cmd)


def _getstring(data, begin, maxlen):
    out = ""
    for i in range(begin, min(begin+maxlen, len(data))):
        if 32 <= data[i] <= 126:
            out += chr(data[i])
        else:
            break
    return out


def _readmem(serport, offset, length):
    cmd = b"\x1b\x05\x08\x00" + struct.pack("<HBB", offset, length, 0) + b"\x6a\x39\x57\x64"
    _send_command(serport, cmd)
    rep = _receive_reply(serport)
    return rep[8:]


def _writemem(serport, data, offset):
    dlen = len(data)
    cmd  = (b"\x1d\x05" +
            struct.pack("<BBHBB", dlen+8, 0, offset, dlen, 1) +
            b"\x6a\x39\x57\x64" + data)
    _send_command(serport, cmd)
    rep = _receive_reply(serport)
    if rep:
        return True
    raise errors.RadioError("No response to writemem at 0x%04X" % offset)


def _sayhello(serport):
    hello = b"\x14\x05\x04\x00\x6a\x39\x57\x64"
    for _ in range(5):
        _send_command(serport, hello)
        try:
            rep = _receive_reply(serport)
        except errors.RadioError:
            continue
        if rep:
            if rep.startswith(b'\x18\x05'):
                raise errors.RadioError("Radio in programming mode — restart into normal mode")
            return _getstring(rep, 4, 24)
    raise errors.RadioError("Radio did not respond")


def _resetradio(serport):
    resetpacket = b"\xdd\x05\x00\x00"
    _send_command(serport, resetpacket)


def do_download(radio):
    serport = radio.pipe; serport.timeout = 4.0
    status = chirp_common.Status()
    status.cur = 0; status.max = MEM_SIZE; status.msg = "Downloading"
    radio.status_fn(status)
    eeprom = b""
    fw = _sayhello(serport)
    if fw: radio.FIRMWARE_VERSION = fw
    addr = 0
    while addr < MEM_SIZE:
        data = _readmem(serport, addr, MEM_BLOCK)
        if data and len(data) == MEM_BLOCK:
            eeprom += data; addr += MEM_BLOCK
        else:
            raise errors.RadioError("Download incomplete")
        status.cur = addr; radio.status_fn(status)
    return memmap.MemoryMapBytes(eeprom)


def do_upload(radio):
    serport = radio.pipe; serport.timeout = 4.0
    status = chirp_common.Status()
    status.cur = 0; status.max = PROG_SIZE; status.msg = "Uploading"
    radio.status_fn(status)
    _sayhello(serport)
    addr = 0
    while addr < PROG_SIZE:
        length = min(MEM_BLOCK, PROG_SIZE - addr)
        chunk  = radio.get_mmap()[addr:addr+length]
        _writemem(serport, chunk, addr)
        addr += length; status.cur = addr; radio.status_fn(status)
    _resetradio(serport)


# ── Mode helpers ─────────────────────────────────────────────────────────────
def _decode_byte0C(b0C, b0D):
    """Decode KA-50 byte 0x0C+0x0D. Returns (pwr, bw, freq_rev, tx_ban, tx_lock, busy_cl)."""
    if b0C & 0x80:
        pwr      = (b0C >> 4) & 0x07
        bw       = (b0C >> 1) & 0x07
        freq_rev = b0C & 0x01
        if b0D & 0x01: bw |= 0x08
        tx_ban   = bool(b0D & 0x02)
        return pwr, bw, freq_rev, tx_ban, False, False
    else:
        tx_lock  = bool((b0C >> 6) & 1)
        busy_cl  = bool((b0C >> 5) & 1)
        pwr_raw  = (b0C >> 2) & 0x07
        pwr      = min(pwr_raw + 1, 4)
        bw       = (b0C >> 1) & 0x01
        freq_rev = b0C & 0x01
        return pwr, bw, freq_rev, False, tx_lock, busy_cl

def _encode_byte0C(pwr, bw, bw_ext, freq_rev, tx_ban):
    """Encode KA-50 new format byte 0x0C and 0x0D (always new format, bit7=1).
    byte0C: bit7=1(new), bits[6:4]=pwr, bits[3:1]=bw[2:0], bit0=freq_rev
    byte0D: bit0=bw_ext, bit1=tx_ban
    """
    b0C = 0x80 | ((pwr & 0x07) << 4) | ((bw & 0x07) << 1) | (freq_rev & 0x01)
    b0D = (0x01 if bw_ext else 0x00) | (0x02 if tx_ban else 0x00)
    return b0C, b0D

def _decode_mode(modulation, bandwidth, bw_ext):
    """Convert KA-50 fields to CHIRP mode string."""
    mod = int(modulation); bw = int(bandwidth); ext = int(bw_ext)
    if mod == 0: return "FM"  if bw == 0 else "NFM"
    if mod == 1: return "AM"  if bw == 0 else "NAM"
    if mod == 2: return "USB"
    if mod == 3: return "CW"
    return "FM"

def _encode_mode(mode_str):
    """Return (modulation, bandwidth, bw_ext) for KA-50."""
    m = mode_str.upper()
    if m == "FM":          return 0, 0, 0
    if m == "NFM":         return 0, 1, 0
    if m == "AM":          return 1, 0, 0
    if m == "NAM":         return 1, 1, 0
    if m in ("DSB","USB"): return 2, 1, 1
    if m == "CW":          return 3, 1, 1
    return 0, 0, 0

def list_def(value, lst, default=0):
    v = int(value)
    return v if v < len(lst) else default

# ── Radio class ──────────────────────────────────────────────────────────────
@directory.register
class KA5052Radio(chirp_common.CloneModeRadio):
    """Quansheng KA-50"""
    VENDOR   = "Quansheng"
    MODEL    = "CHIRP_KA-50/52"
    BAUD_RATE = 38400
    NEEDS_COMPAT_SERIAL = False
    FIRMWARE_VERSION = ""
    upload_calibration = False

    def _get_bands(self):
        wide = self._memobj.BUILD_OPTIONS.ENABLE_WIDE_RX \
               if self._memobj is not None else True
        return BANDS_WIDE if wide else BANDS_STANDARD

    def _find_band(self, hz):
        mhz = hz / 1e6
        for bnd, rng in self._get_bands().items():
            if rng[0] <= mhz <= rng[1]: return bnd
        return False

    def _vfo_names(self):
        names = []
        for bnd, rng in self._get_bands().items():
            n = f"F{bnd+1}({round(rng[0])}M-{round(rng[1])}M)"
            names += [n+"A", n+"B"]
        return names

    def _get_specials(self):
        return {n: MR_CHANNELS_MAX+i for i, n in enumerate(self._vfo_names())}

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.pre_download = ("1. Turn radio on.\n"
                           "2. Connect Kenwood cable or USB Type-C.\n"
                           "3. Click OK.\n")
        rp.pre_upload = rp.pre_download
        return rp

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.valid_dtcs_codes = DTCS_CODES
        rf.has_rx_dtcs = True; rf.has_ctone = True
        rf.has_settings = True; rf.has_comment = False
        rf.valid_name_length = 10
        rf.valid_power_levels = UVK5_POWER_LEVELS
        rf.valid_special_chans = self._vfo_names()
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_tuning_steps = sorted(STEPS)
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone","Tone->DTCS","DTCS->Tone",
                                "->Tone","->DTCS","DTCS->","DTCS->DTCS"]
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_modes = VALID_MODES
        rf.valid_skips = [""]
        rf.memory_bounds = (1, MR_CHANNELS_MAX)
        rf.valid_bands = [(int(r[0]*1e6), int(r[1]*1e6))
                          for r in self._get_bands().values()]
        return rf

    def load_mmap(self, filename):
        with open(filename, "rb") as f:
            data = bytearray(f.read())
        if len(data) < MEM_SIZE:
            raise errors.RadioError("Файл слишком мал: %d байт" % len(data))
        # Обрезаем до MEM_SIZE (калибровки и метаданные отбрасываем)
        self._mmap = memmap.MemoryMapBytes(bytes(data[:MEM_SIZE]))
        self.process_mmap()

    @classmethod
    def match_model(cls, filedata, filename):
        if len(filedata) < MEM_SIZE:
            return False
        ver = filedata[0xA160:0xA161]
        return ver in (b'v', b'V')  # KA-50: v4.x  KA-52: V.5.x

    def sync_in(self):
        self._mmap = do_download(self); self.process_mmap()

    def sync_out(self):
        do_upload(self)

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def get_raw_memory(self, number):
        return repr(self._memobj.channel[number-1])

    # ── Tone ─────────────────────────────────────────────────────────────────
    def _set_tone(self, mem, _mem):
        ((txm,txt,txp),(rxm,rxt,rxp)) = chirp_common.split_tone_encode(mem)
        for mode,tone,pol,ca,fa in [(txm,txt,txp,"txcode","txcodeflag"),
                                     (rxm,rxt,rxp,"rxcode","rxcodeflag")]:
            if mode=="Tone":
                if tone in CTCSS_TONES:
                    setattr(_mem,fa,0b01); setattr(_mem,ca,CTCSS_TONES.index(tone))
                else:
                    setattr(_mem,fa,0b00); setattr(_mem,ca,0)
            elif mode=="DTCS":
                setattr(_mem,fa,0b11 if pol=="R" else 0b10)
                setattr(_mem,ca,DTCS_CODES.index(tone))
            else:
                setattr(_mem,fa,0); setattr(_mem,ca,0)

    def _get_tone(self, mem, _mem):
        rxt = int(_mem.rxcodeflag); txt = int(_mem.txcodeflag)
        if rxt >= len(TMODES): rxt = 0
        if txt >= len(TMODES): txt = 0
        rxm = TMODES[rxt]; txm = TMODES[txt]
        tx_t = rx_t = None
        if txm=="Tone":
            i=int(_mem.txcode); tx_t=CTCSS_TONES[i] if i<len(CTCSS_TONES) else None
        elif txm=="DTCS":
            i=int(_mem.txcode); tx_t=DTCS_CODES[i] if i<len(DTCS_CODES) else None
        if rxm=="Tone":
            i=int(_mem.rxcode); rx_t=CTCSS_TONES[i] if i<len(CTCSS_TONES) else None
        elif rxm=="DTCS":
            i=int(_mem.rxcode); rx_t=DTCS_CODES[i] if i<len(DTCS_CODES) else None
        chirp_common.split_tone_decode(mem,
            (txm,tx_t,"R" if txt==3 else "N"),
            (rxm,rx_t,"R" if rxt==3 else "N"))

    # ── get_memory ───────────────────────────────────────────────────────────

    def validate_memory(self, mem):
        msgs = chirp_common.CloneModeRadio.validate_memory(self, mem)
        if mem.mode not in VALID_MODES:
            msgs.append(chirp_common.ValidationError(
                f"Mode {mem.mode!r} not supported; use one of {VALID_MODES}"))
        return msgs

    def get_memory(self, number):
        mem = chirp_common.Memory()
        if isinstance(number, str):
            ch_num = self._get_specials()[number]; mem.extd_number = number
        else:
            ch_num = number - 1
        mem.number = ch_num + 1

        _mem = (self._memobj.channel[ch_num] if ch_num < MR_CHANNELS_MAX
                else self._memobj.vfo_channel[ch_num - MR_CHANNELS_MAX])

        if int(_mem.freq) in (0xFFFFFFFF, 0):
            mem.empty = True; mem.power = UVK5_POWER_LEVELS[0]
            mem.extra = RadioSettingGroup("Extra", "extra")
            mem.extra.append(RadioSetting("busyChLockout", "BusyCL",
                RadioSettingValueBoolean(False)))
            mem.extra.append(RadioSetting("frev", "FreqRev",
                RadioSettingValueBoolean(False)))
            mem.extra.append(RadioSetting("tx_ban", "TX Forbidden",
                RadioSettingValueBoolean(False)))
            mem.extra.append(RadioSetting("compander", "Compander",
                RadioSettingValueList(COMPANDER_LIST)))
            mem.extra.append(RadioSetting("scanlists", "Scanlists",
                RadioSettingValueList(SCANLIST_LIST)))
            return mem

        if ch_num >= MR_CHANNELS_MAX:
            mem.name = self._vfo_names()[ch_num - MR_CHANNELS_MAX]
            mem.immutable = ["name","scanlists"]
        else:
            _mn = self._memobj.channelname[ch_num]
            for c in _mn.name:
                if str(c) in ("\xFF","\x00"): break
                mem.name += str(c)
            mem.name = mem.name.rstrip().rstrip("\xff").rstrip().rstrip("\xff").rstrip()

        mem.freq   = int(_mem.freq)   * 10
        mem.offset = int(_mem.offset) * 10
        mem.duplex = {FLAGS1_OFFSET_MINUS:"-",
                      FLAGS1_OFFSET_PLUS: "+"}.get(int(_mem.offsetDir),"")
        self._get_tone(mem, _mem)

        _b0C = int(_mem.byte0C); _b0D = int(_mem.byte0D)
        _pwr, _bw, _frev, _tx_ban, _tx_lock, _busy_cl = _decode_byte0C(_b0C, _b0D)

        _bw_ext = 1 if (_b0D & 0x01) else 0  # bit0 of byte0D = bw_ext for CW/DSB
        mem.mode = _decode_mode(_mem.modulation, _bw, _bw_ext)

        ts = int(_mem.step)
        mem.tuning_step = STEPS[ts] if ts < len(STEPS) else 2.5

        # _pwr from _decode_byte0C: 0=X,1=L,2=M,3=H,4=U
        tx_off = (_pwr == 0) or _tx_ban
        _pwr_map = {1: 0, 2: 1, 3: 2, 4: 3}
        mem.power = UVK5_POWER_LEVELS[_pwr_map.get(_pwr, 0)]

        mem.extra = RadioSettingGroup("Extra", "extra")

        tmp_comp = 0; tmpscn = 0
        if ch_num < MR_CHANNELS_MAX:
            _a = self._memobj.ch_attr[ch_num]
            tmp_comp = list_def(_a.compander, COMPANDER_LIST)
            tmpscn   = min(int(_a.scanlist), len(SCANLIST_LIST)-1)

        mem.extra.append(RadioSetting("busyChLockout", "BusyCL",
            RadioSettingValueBoolean(_busy_cl)))
        mem.extra.append(RadioSetting("frev", "FreqRev",
            RadioSettingValueBoolean(bool(_frev))))
        mem.extra.append(RadioSetting("tx_ban", "TX Forbidden",
            RadioSettingValueBoolean(_tx_ban)))
        mem.extra.append(RadioSetting("compander", "Compander",
            RadioSettingValueList(COMPANDER_LIST, None, tmp_comp)))
        mem.extra.append(RadioSetting("scanlists", "Scanlists",
            RadioSettingValueList(SCANLIST_LIST, None, tmpscn)))
        return mem

    # ── set_memory ───────────────────────────────────────────────────────────
    def set_memory(self, memory):
        ch_num = memory.number - 1
        _mem  = (self._memobj.channel[ch_num] if ch_num < MR_CHANNELS_MAX
                 else self._memobj.vfo_channel[ch_num - MR_CHANNELS_MAX])
        _attr = self._memobj.ch_attr[ch_num] if ch_num < MR_CHANNELS_MAX else None

        if memory.empty:
            _mem.set_raw(b"\xFF"*16)
            if _attr: _attr.set_raw(b"\xFF\xFF")
            return memory

        def ex(n, d):
            return int(memory.extra[n].value) if n in memory.extra else d

        mod, bw, bw_ext = _encode_mode(memory.mode)
        _mem.modulation = mod

        _p = str(memory.power)
        _pwr_enc = (OUTPUT_POWER_HIGH if _p == "HIGH"
                    else OUTPUT_POWER_MID  if _p == "MID"
                    else OUTPUT_POWER_USER if _p == "USER"
                    else OUTPUT_POWER_LOW)
        # tx_ban: из extra поля ИЛИ из duplex="off"
        _tx_ban_enc = bool(ex("tx_ban", 0)) or (memory.duplex == "off")
        _frev_enc   = ex("frev", 0)
        # _encode_byte0C(pwr, bw, bw_ext, freq_rev, tx_ban)
        _b0C, _b0D = _encode_byte0C(_pwr_enc, bw, bw_ext, _frev_enc, _tx_ban_enc)
        _mem.byte0C = _b0C
        _mem.byte0D = _b0D

        _mem.offsetDir = {"-":FLAGS1_OFFSET_MINUS,
                          "+":FLAGS1_OFFSET_PLUS}.get(memory.duplex, FLAGS1_OFFSET_NONE)
        _mem.freq   = memory.freq   // 10
        _mem.offset = memory.offset // 10

        band = self._find_band(memory.freq)
        if band is not False and _attr: _attr.band = band

        if ch_num < MR_CHANNELS_MAX:
            self._memobj.channelname[ch_num].name = \
                memory.name.ljust(10)[:10] + "\x00"*6
            if _attr:
                _attr.compander = ex("compander", 0)
                _attr.scanlist  = ex("scanlists",  0)

        self._set_tone(memory, _mem)
        try:
            _mem.step = STEPS.index(memory.tuning_step)
        except ValueError:
            # Find nearest step
            nearest = min(STEPS, key=lambda s: abs(s - memory.tuning_step))
            _mem.step = STEPS.index(nearest)
        return memory

    # ── get_settings ─────────────────────────────────────────────────────────
    def get_settings(self):
        _mem = self._memobj
        fw = self.FIRMWARE_VERSION or "read from radio"

        basic = RadioSettingGroup("basic",  "Basic Settings")
        adv   = RadioSettingGroup("adv",    "Advanced Settings")
        keya  = RadioSettingGroup("keya",   "Programmable Keys")
        fmr   = RadioSettingGroup("fmradio","FM Radio (2 Banks × 6 ch)")
        cal   = RadioSettingGroup("cal",    "Calibration (read-only)")
        top   = RadioSettings()
        top.append(basic); top.append(adv); top.append(keya)
        if _mem.BUILD_OPTIONS.ENABLE_FMRADIO: top.append(fmr)
        top.append(cal)

        lbl_idx = [0]
        def lbl(grp, text, desc=""):
            v = RadioSettingValueString(max(1,len(desc)),max(1,len(desc)),desc)
            v.set_mutable(False)
            grp.append(RadioSetting(f"lbl{lbl_idx[0]}", text, v))
            lbl_idx[0] += 1

        # Keys
        def get_action(raw):
            raw = int(raw)
            cur = (KEYACTIONS_LIST[raw]
                   if raw < len(KEYACTIONS_LIST) and KEYACTIONS_LIST[raw]
                   else "NONE")
            return KEYACTIONS_VISIBLE, cur

        for attr, label in [
            ("key1_shortpress_action","Side Key 1 Short (F1Shrt)"),
            ("key1_longpress_action", "Side Key 1 Long  (F1Long)"),
            ("key2_shortpress_action","Side Key 2 Short (F2Shrt)"),
            ("key2_longpress_action", "Side Key 2 Long  (F2Long)"),
            ("keyM_longpress_action", "Menu Key Long    (M Long)"),
        ]:
            val = RadioSettingValueList(*get_action(getattr(_mem, attr)))
            keya.append(RadioSetting(attr, label, val))

        # Basic
        def li(g,a,lb,lst):
            v=RadioSettingValueList(lst,None,list_def(getattr(_mem,a),lst))
            g.append(RadioSetting(a,lb,v))
        def bi(g,a,lb):
            g.append(RadioSetting(a,lb,RadioSettingValueBoolean(bool(getattr(_mem,a)))))
        def ii(g,a,lb,mn,mx):
            g.append(RadioSetting(a,lb,RadioSettingValueInteger(mn,mx,int(getattr(_mem,a)))))

        li(basic,"TX_VFO",            "Active VFO",          TX_VFO_LIST)
        ii(basic,"squelch",           "Squelch (SQL)",        0, 9)
        li(basic,"channel_display_mode","Channel Display",    CHANNELDISP_LIST)
        li(basic,"battery_save",      "Battery Save",         BATSAVE_LIST)
        li(basic,"backlight_time",    "Backlight Time",       BACKLIGHT_LIST)
        li(basic,"backlight_on_TX_RX","Backlight on TX/RX",  BL_TX_RX_LIST)
        li(basic,"battery_text",      "Battery Display",      BAT_TXT_LIST)
        li(basic,"Battery_type",      "Battery Type",         BATTYPE_LIST)
        li(basic,"power_on_dispmode", "Power On Message",     WELCOME_LIST)
        li(basic,"roger_beep",        "Roger Tone",           ROGER_LIST)
        li(basic,"scan_resume_mode",  "Scan Resume",          SCANRESUME_LIST)

        for a,lb in [("logo_line1","Welcome Line 1"),("logo_line2","Welcome Line 2")]:
            v=RadioSettingValueString(0,16,
                str(getattr(_mem,a)).rstrip("\xFF\x00").strip())
            basic.append(RadioSetting(a,lb,v))

        # Advanced
        bi(adv,"AM_fix",    "AM Fix")
        bi(adv,"mic_bar",   "MIC Bar")
        bi(adv,"button_beep","Button Beep")
        li(adv,"mic_gain",  "MIC Gain",       MIC_GAIN_LIST)
        li(adv,"alarm_mode","Alarm Mode",      ALARMMODE_LIST)
        li(adv,"int_flock", "Freq Lock",       FLOCK_LIST)

        # FM Radio
        lbl(fmr,"Bank 1 (ch 1-6)","MHz 76.0-108.0")
        for i in range(4):
            raw=int(_mem.fm_bank1[i])
            f=raw/10.0 if FMMIN*10<=raw<=FMMAX*10 else FMMIN
            fmr.append(RadioSetting(f"fm_b1_{i}",f"Bank1 Ch{i+1}",
                RadioSettingValueString(0,7,f"{f:.1f}")))
        for i in range(2):
            raw=int(_mem.fm_bank1_45[i])
            f=raw/10.0 if FMMIN*10<=raw<=FMMAX*10 else FMMIN
            fmr.append(RadioSetting(f"fm_b1_{i+4}",f"Bank1 Ch{i+5}",
                RadioSettingValueString(0,7,f"{f:.1f}")))
        lbl(fmr,"Bank 2 (ch 1-6)","MHz 76.0-108.0")
        for i in range(4):
            raw=int(_mem.fm_bank2[i])
            f=raw/10.0 if FMMIN*10<=raw<=FMMAX*10 else FMMIN
            fmr.append(RadioSetting(f"fm_b2_{i}",f"Bank2 Ch{i+1}",
                RadioSettingValueString(0,7,f"{f:.1f}")))
        for i in range(2):
            raw=int(_mem.fm_bank2_45[i])
            f=raw/10.0 if FMMIN*10<=raw<=FMMAX*10 else FMMIN
            fmr.append(RadioSetting(f"fm_b2_{i+4}",f"Bank2 Ch{i+5}",
                RadioSettingValueString(0,7,f"{f:.1f}")))

        # Calibration read-only
        lbl(cal,"Calibration 0xB000-0xB190","Edit with UV AIR TOOL")
        for a,lb,mn,mx in [("xtalFreqLow","Crystal Offset",-32768,32767),
                             ("volumeGain","Volume Gain",0,255),
                             ("dacGain","DAC Gain",0,255)]:
            v=RadioSettingValueInteger(mn,mx,int(getattr(_mem.cal,a)))
            v.set_mutable(False)
            cal.append(RadioSetting(f"cal_{a}",lb,v))
        for i in range(7):
            v=RadioSettingValueInteger(-128,127,int(_mem.dbm_corr[i]))
            v.set_mutable(False)
            cal.append(RadioSetting(f"dbm_corr_{i}",f"dBm Corr Band {i+1}",v))

        return top

    # ── set_settings ─────────────────────────────────────────────────────────
    def set_settings(self, settings):
        _mem = self._memobj
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element); continue
            if not element.changed(): continue
            nm = element.get_name()
            if nm.startswith("lbl") or nm.startswith("cal_") or nm.startswith("dbm_corr"):
                continue
            try:
                if nm.startswith("fm_b1_"):
                    idx=int(nm[6:])
                    try:
                        f=float(str(element.value))
                        if FMMIN<=f<=FMMAX:
                            if idx<4: _mem.fm_bank1[idx]=int(f*10)
                            else: _mem.fm_bank1_45[idx-4]=int(f*10)
                    except ValueError: pass
                    continue
                if nm.startswith("fm_b2_"):
                    idx=int(nm[6:])
                    try:
                        f=float(str(element.value))
                        if FMMIN<=f<=FMMAX:
                            if idx<4: _mem.fm_bank2[idx]=int(f*10)
                            else: _mem.fm_bank2_45[idx-4]=int(f*10)
                    except ValueError: pass
                    continue
                if nm.endswith("_action"):
                    n=str(element.value)
                    if n in KEYACTIONS_LIST:
                        setattr(_mem,nm,KEYACTIONS_LIST.index(n))
                    continue
                if nm in ("logo_line1","logo_line2"):
                    s = str(element.value).strip()[:16]
                    setattr(_mem, nm, (s + "\x00" * 16)[:16]); continue
                if nm in ("backlight_min","backlight_max"):
                    setattr(_mem,nm,int(str(element.value))); continue
                v = element.value
                if isinstance(v, RadioSettingValueBoolean):
                    setattr(_mem,nm,bool(v))
                elif isinstance(v, RadioSettingValueInteger):
                    setattr(_mem,nm,int(v))
                elif isinstance(v, RadioSettingValueList):
                    sv=str(v)
                    for lst in [TX_VFO_LIST,CHANNELDISP_LIST,BATSAVE_LIST,
                                 BACKLIGHT_LIST,BL_TX_RX_LIST,BAT_TXT_LIST,
                                 BATTYPE_LIST,WELCOME_LIST,ROGER_LIST,
                                 SCANRESUME_LIST,MIC_GAIN_LIST,
                                 ALARMMODE_LIST,FLOCK_LIST,BL_LVL_LIST]:
                        if sv in lst:
                            setattr(_mem,nm,lst.index(sv)); break
                elif isinstance(v, RadioSettingValueString):
                    setattr(_mem,nm,str(v))
            except Exception as e:
                LOG.warning(f"set_settings {nm}: {e}")
