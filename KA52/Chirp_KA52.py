import webbrowser
import os

import struct
import logging
import wx


from chirp import chirp_common, directory, bitwise, memmap, errors, util
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueBoolean, RadioSettingValueList, \
    RadioSettingValueInteger, RadioSettingValueString, \
    RadioSettings, InvalidValueError

LOG = logging.getLogger(__name__)

# Show the obfuscated version of commands. Not needed normally, but
# might be useful for someone who is debugging a similar radio
DEBUG_SHOW_OBFUSCATED_COMMANDS = False

# Show the memory being written/received. Not needed normally, because
# this is the same information as in the packet hexdumps, but
# might be useful for someone debugging some obscure memory issue
DEBUG_SHOW_MEMORY_ACTIONS = False

# TODO: remove the driver version when it's in mainline chirp 
DRIVER_VERSION = "Quansheng KA-52 CHIRP driver 2026/06/14 (c) OUROBOROS"
FIRMWARE_VERSION_UPDATE = "https://github.com/armel/uv-k1-k5v3-firmware-custom/releases"
CHIRP_DRIVER_VERSION_UPDATE = "https://github.com/armel/uv-k1-k5v3-chirp-driver/releases"

VALEUR_COMPILER = "ENABLE"

MEM_FORMAT = """
// --------------------

#seekto 0x000000;
struct {
  ul32 freq;
  ul32 offset;

// 0x08
  u8 rxcode;
  u8 txcode;

// 0x0A
  u8 txcodeflag:4,
  rxcodeflag:4;

// 0x0B
  u8 modulation:4,
  offsetDir:4;

// 0x0C  bit0=frev, bit1=bw, bits[4:2]=pwr, bit5=busyCL, bit7-6=unused
  u8 freq_reverse:1,
  bandwidth:1,
  txpower:3,
  busyChLockout:1,
  __UNUSED01:2;

  // 0x0D  bit0=TX_BAN
  u8 __UNUSED02:4,
  dtmf_pttid:3,
  tx_ban:1;

  // 0x0E
  u8 step;
  u8 __UNUSED03;

} channel[1024]; //end 0x3FFF

// --------------------

#seekto 0x004000;
struct {
char name[16];
} channelname[1024]; //end 0x7FFF


// --------------------

#seekto 0x008000;
struct {
  u8 __UNUSED04:3,
     compander:2,
     band:3;
  u8 scanlist;
} ch_attr[1031]; //end 0x00880D

// --------------------

#seekto 0x00880E;
struct {
    char name[4];
} listname[24]; //end 886D

// --------------------

#seekto 0x009000;
struct {
  ul32 freq;
  ul32 offset;

// 0x08
  u8 rxcode;
  u8 txcode;

// 0x0A
  u8 txcodeflag:4,
  rxcodeflag:4;

// 0x0B
  u8 modulation:4,
  offsetDir:4;

// 0x0C
  u8 __UNUSED05:1,
  txLock:1,
  busyChLockout:1,
  txpower:3,
  bandwidth:1,
  freq_reverse:1;

  // 0x0D
  u8 __UNUSED06:4,
  dtmf_pttid:3,
  dtmf_decode:1;

  // 0x0E
  u8 step;
  u8 __UNUSED07;

} vfo_channel[14];

// --------------------

#seekto 0x00A000;
u8 set_rxa;
u8 squelch;
u8 max_talk_time;
u8 noaa_autoscan;
u8 __UNUSED09:1,
   set_nav:1,
   set_key:4,
   set_menu_lock:1,
   key_lock:1;
u8 vox_switch;
u8 vox_level;
u8 mic_gain;

// --------------------

#seekto 0x00A008;
u8 backlight_min:4,
   backlight_max:4;

u8 channel_display_mode;
u8 crossband;
u8 battery_save;
u8 dual_watch;
u8 backlight_time;
u8 __UNUSED10:5,
   set_nfm:2,
   ste:1;
u8 current_state;

// --------------------

#seekto 0x00A010;
ul16 ScreenChannel_A;
ul16 MrChannel_A;
ul16 FreqChannel_A;
ul16 ScreenChannel_B;
ul16 MrChannel_B;
ul16 FreqChannel_B;
ul16 NoaaChannel_A;
ul16 NoaaChannel_B;

// --------------------

#seekto 0x00A070;
ul16 fmfreq[6];

// --------------------

#seekto 0x00A0A8;
u8 keyM_longpress_action:7,
   button_beep:1;

u8 key1_shortpress_action;
u8 key1_longpress_action;
u8 key2_shortpress_action;
u8 key2_longpress_action;
u8 scan_resume_mode;    
u8 auto_keypad_lock;
u8 power_on_dispmode;
ul32 password;

// --------------------

#seekto 0x00A0B8;
u8 voice;
i8 dbm_corr[7];

// --------------------

#seekto 0x00A0C0;
u8 alarm_mode;
u8 roger_beep;
u8 rp_ste;
u8 TX_VFO;
u8 Battery_type;

// --------------------

#seekto 0x00A0C8;
char logo_line1[16];
char logo_line2[16];

// --------------------

#seekto 0x00A0E8;
struct {
    u8 side_tone;
    char separate_code;
    char group_call_code;
    u8 decode_response;
    u8 auto_reset_time;
    u8 preload_time;
    u8 first_code_persist_time;
    u8 hash_persist_time;
    u8 code_persist_time;
    u8 code_interval_time;
    u8 permit_remote_kill;

    #seekto 0x00A0F8;
    char local_code[3];
    #seek 5;
    char kill_code[5];
    #seek 3;
    char revive_code[5];
    #seek 3;
    char up_code[16];
    char down_code[16];
} dtmf;

// --------------------

#seekto 0x00A130;

struct {
    u8 slPriorEnab:1,
       slDef:7;
        
    ul16 slPriorCh1;
    ul16 slPriorCh2;
    ul16 call_channel;

    u8 __UNUSED11;
} sl;

// --------------------

#seekto 0x00A150;
u8 int_flock;
u8 int_350tx_unsused;
u8 int_KILLED;
u8 int_200tx_unsused;
u8 int_500tx_unsused;
u8 int_350en;
u8 int_scren;

u8  backlight_on_TX_RX:2,
    AM_fix:1,
    mic_bar:1,
    battery_text:2,
    live_DTMF_decoder:1,
    __UNUSED12:1;

// --------------------

#seekto 0x00A158;
struct {
u8 ENABLE_DTMF_CALLING:1,
   ENABLE_PWRON_PASSWORD:1,
   ENABLE_TX1750:1,
   ENABLE_ALARM:1,
   ENABLE_VOX:1,
   ENABLE_VOICE:1,
   ENABLE_NOAA:1,
   ENABLE_FMRADIO:1;
u8 __UNUSED13:1,
   ENABLE_FEAT_F4HWN_RESCUE_OPS:1,
   ENABLE_BANDSCOPE:1,
   ENABLE_AM_FIX:1,
   ENABLE_FEAT_F4HWN_GAME:1,
   ENABLE_RAW_DEMODULATORS:1,
   ENABLE_WIDE_RX:1,
   ENABLE_FLASHLIGHT:1;
} BUILD_OPTIONS;

u8 __UNUSED14;
u8 __UNUSED15;

u8 set_off_tmr:7,
set_tmr:1;

u8 set_gui:1,
set_met:1,
set_lck:1,
set_inv:1,
set_contrast:4;

u8 set_tot:4,
set_eot:4;

u8 set_pwr:4,
set_ptt:4;

#seekto 0x00A160;
struct {
    char version[16];
} version;

// --------------------

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
    struct {
        ul16 level1;
        ul16 level2;
        ul16 level4;
        ul16 level6;
    } rssiLevelsBands3_7;

    struct {
        ul16 level1;
        ul16 level2;
        ul16 level4;
        ul16 level6;
    } rssiLevelsBands1_2;

    struct {
        struct {
            u8 lower;
            u8 center;
            u8 upper;
        } low;
        struct {
            u8 lower;
            u8 center;
            u8 upper;
        } mid;
        struct {
            u8 lower;
            u8 center;
            u8 upper;
        } hi;
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
# SONIC parameter
FM_CHANNELS_MAX = 6
MR_CHANNELS_MAX = 1024
MR_CHANNELS_LIST = 21

# flags1
FLAGS1_OFFSET_NONE = 0b00
FLAGS1_OFFSET_MINUS = 0b10
FLAGS1_OFFSET_PLUS = 0b01


POWER_TX_OFF =  0b000   # KA-52: 0 = TX forbidden
POWER_LOW    =  0b001   # L
POWER_MEDIUM =  0b010   # M
POWER_HIGH   =  0b011   # H
POWER_ULTRA  =  0b100   # U (User)

SET_LOW_LIST = ["L", "M", "H", "U"]

# SET_TOT and SET_EOT SONIC
SET_TOT_EOT_LIST = ["OFF", "SOUND", "VISUAL", "ALL"]

# SET_OFF_ON SONIC
SET_OFF_ON_LIST = ["OFF", "ON"]

# SET_lck SONIC
SET_LCK_LIST = ["KEYS", "KEYS+PTT"]

# SET_MET SET_GUI SONIC
SET_MET_LIST = ["TINY", "CLASSIC"]

# dtmf_flags
PTTID_LIST = ["OFF", "UP CODE", "DOWN CODE", "UP+DOWN CODE", "APOLLO QUINDAR"]

# power          
UVK5_POWER_LEVELS = [
    chirp_common.PowerLevel("L", watts=0.5),
    chirp_common.PowerLevel("M", watts=2.0),
    chirp_common.PowerLevel("H", watts=5.0),
    chirp_common.PowerLevel("U", watts=8.0),
]

# compander
COMPANDER_LIST = ["OFF", "TX", "RX", "TX/RX"]

# rx mode
RXMODE_LIST = ["MAIN ONLY", "DUAL RX RESPOND", "CROSS BAND", "MAIN TX DUAL RX"]

# channel display mode
CHANNELDISP_LIST = ["FREQ", "CHANNEL NUMBER", "NAME", "NAME+FREQ"]

# TalkTime
TALK_TIME_LIST = ["N/U", "N/U", "N/U", "N/U", "N/U", "30 sec", "35 sec", "40 sec", "45 sec", "50 sec", "55 sec", 
                  "1 min", "1 min : 5 sec", "1 min : 10 sec", "1 min : 15 sec", "1 min : 20 sec", "1 min : 25 sec", "1 min : 30 sec", "1 min : 35 sec", "1 min : 40 sec", "1 min : 45 sec", "1 min : 50 sec", "1 min : 55 sec", 
                  "2 min", "2 min : 5 sec", "2 min : 10 sec", "2 min : 15 sec", "2 min : 20 sec", "2 min : 25 sec", "2 min : 30 sec", "2 min : 35 sec", "2 min : 40 sec", "2 min : 45 sec", "2 min : 50 sec", "2 min : 55 sec", 
                  "3 min", "3 min : 5 sec", "3 min : 10 sec", "3 min : 15 sec", "3 min : 20 sec", "3 min : 25 sec", "3 min : 30 sec", "3 min : 35 sec", "3 min : 40 sec", "3 min : 45 sec", "3 min : 50 sec", "3 min : 55 sec", 
                  "4 min", "4 min : 5 sec", "4 min : 10 sec", "4 min : 15 sec", "4 min : 20 sec", "4 min : 25 sec", "4 min : 30 sec", "4 min : 35 sec", "4 min : 40 sec", "4 min : 45 sec", "4 min : 50 sec", "4 min : 55 sec",
                  "5 min", "5 min : 5 sec", "5 min : 10 sec", "5 min : 15 sec", "5 min : 20 sec", "5 min : 25 sec", "5 min : 30 sec", "5 min : 35 sec", "5 min : 40 sec", "5 min : 45 sec", "5 min : 50 sec", "5 min : 55 sec",
                  "6 min", "6 min : 5 sec", "6 min : 10 sec", "6 min : 15 sec", "6 min : 20 sec", "6 min : 25 sec", "6 min : 30 sec", "6 min : 35 sec", "6 min : 40 sec", "6 min : 45 sec", "6 min : 50 sec", "6 min : 55 sec", 
                  "7 min", "7 min : 5 sec", "7 min : 10 sec", "7 min : 15 sec", "7 min : 20 sec", "7 min : 25 sec", "7 min : 30 sec", "7 min : 35 sec", "7 min : 40 sec", "7 min : 45 sec", "7 min : 50 sec", "7 min : 55 sec", 
                  "8 min", "8 min : 5 sec", "8 min : 10 sec", "8 min : 15 sec", "8 min : 20 sec", "8 min : 25 sec", "8 min : 30 sec", "8 min : 35 sec", "8 min : 40 sec", "8 min : 45 sec", "8 min : 50 sec", "8 min : 55 sec", 
                  "9 min", "9 min : 5 sec", "9 min : 10 sec", "9 min : 15 sec", "9 min : 20 sec", "9 min : 25 sec", "9 min : 30 sec", "9 min : 35 sec", "9 min : 40 sec", "9 min : 45 sec", "9 min : 50 sec", "9 min : 55 sec",
                  "10 min", "10 min : 5 sec", "10 min : 10 sec", "10 min : 15 sec", "10 min : 20 sec", "10 min : 25 sec", "10 min : 30 sec", "10 min : 35 sec", "10 min : 40 sec", "10 min : 45 sec", "10 min : 50 sec", "10 min : 55 sec",
                  "11 min", "11 min : 5 sec", "11 min : 10 sec", "11 min : 15 sec", "11 min : 20 sec", "11 min : 25 sec", "11 min : 30 sec", "11 min : 35 sec", "11 min : 40 sec", "11 min : 45 sec", "11 min : 50 sec", "11 min : 55 sec", 
                  "12 min", "12 min : 5 sec", "12 min : 10 sec", "12 min : 15 sec", "12 min : 20 sec", "12 min : 25 sec", "12 min : 30 sec", "12 min : 35 sec", "12 min : 40 sec", "12 min : 45 sec", "12 min : 50 sec", "12 min : 55 sec", 
                  "13 min", "13 min : 5 sec", "13 min : 10 sec", "13 min : 15 sec", "13 min : 20 sec", "13 min : 25 sec", "13 min : 30 sec", "13 min : 35 sec", "13 min : 40 sec", "13 min : 45 sec", "13 min : 50 sec", "13 min : 55 sec", 
                  "14 min", "14 min : 5 sec", "14 min : 10 sec", "14 min : 15 sec", "14 min : 20 sec", "14 min : 25 sec", "14 min : 30 sec", "14 min : 35 sec", "14 min : 40 sec", "14 min : 45 sec", "14 min : 50 sec", "14 min : 55 sec",
                  "15 min"]

# Set NFM value
SET_NFM_LIST = ["NARROW", "NARROWER"]

# Set RxA value
SET_RXA_LIST = ["FLAT", "CLEAN", "MID", "BOOST", "MAX"]

# Set KEY value
SET_KEY_LIST = ["MENU", "KEY_UP", "KEY_DOWN", "KEY_EXIT", "KEY_STAR"]

# Set Off timer 
SET_OFF_TMR_LIST = ["OFF"]

# Add values from 00h:01m to 02h:00m
for h in range(2):  # From 0 to 2 hours
    if h == 1:  # Add 01h:00m
        SET_OFF_TMR_LIST.append(f"{h:d}h:00m")
    for m in range(1, 60):  # From 1 to 59 minutes (start at 1)
        SET_OFF_TMR_LIST.append(f"{h:d}h:{m:02d}m")

SET_OFF_TMR_LIST.append(f"2h:00m")

# Add Auto Keypad Lock values
AUTO_KEYPAD_LOCK_LIST = ["OFF"]
for s in range(10):  # From 0 to 10 minutes
    for ms in ["00s", "15s", "30s", "45s"]:
        if s == 0 and ms == "00s":  # Cancel "00m:00s"
            continue
        AUTO_KEYPAD_LOCK_LIST.append(f"{s:02d}m:{ms}")
AUTO_KEYPAD_LOCK_LIST.append("10m:00s")  # Add "10m:00s" a the end

# set nav
SET_NAV_LIST = ["LEFT/RIGHT (UV-K1)", "UP/DOWN (UV-K5 V3)"]

# battery save
BATSAVE_LIST = ["OFF", "1:1", "1:2", "1:3", "1:4", "1:5"]

# battery type
BATTYPE_LIST = ["1400mAh UV-K1", "2500mAh UV-K1", "1600mAh UV-K5", "2200mAh UV-R5+", "3500mAh UV-K5"]
# bat txt
BAT_TXT_LIST = ["NONE", "VOLTAGE", "PERCENT"]
# Backlight auto mode
BACKLIGHT_LIST = ["OFF", "5 sec", "10 sec", "15 sec", "20 sec", "25 sec", "30 sec", "35 sec", "40 sec", "45 sec", "50 sec", "55 sec", 
                  "1 min", "1 min : 5 sec", "1 min : 10 sec", "1 min : 15 sec", "1 min : 20 sec", "1 min : 25 sec", "1 min : 30 sec", "1 min : 35 sec", "1 min : 40 sec", "1 min : 45 sec", "1 min : 50 sec", "1 min : 55 sec", 
                  "2 min", "2 min : 5 sec", "2 min : 10 sec", "2 min : 15 sec", "2 min : 20 sec", "2 min : 25 sec", "2 min : 30 sec", "2 min : 35 sec", "2 min : 40 sec", "2 min : 45 sec", "2 min : 50 sec", "2 min : 55 sec", 
                  "3 min", "3 min : 5 sec", "3 min : 10 sec", "3 min : 15 sec", "3 min : 20 sec", "3 min : 25 sec", "3 min : 30 sec", "3 min : 35 sec", "3 min : 40 sec", "3 min : 45 sec", "3 min : 50 sec", "3 min : 55 sec", 
                  "4 min", "4 min : 5 sec", "4 min : 10 sec", "4 min : 15 sec", "4 min : 20 sec", "4 min : 25 sec", "4 min : 30 sec", "4 min : 35 sec", "4 min : 40 sec", "4 min : 45 sec", "4 min : 50 sec", "4 min : 55 sec",
                  "5 min", "Always On (ON)"]

# Backlight LVL
BACKLIGHT_LVL_LIST = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]

# Backlight _TX_RX_LIST
BACKLIGHT_TX_RX_LIST = ["OFF", "TX", "RX", "TX/RX"]


STEPS = [0.01, 0.05, 0.1, 0.25, 0.5, 1, 1.25, 2.5, 5, 6.25, 8.33, 9, 10, 12.5, 15, 20, 25, 30, 50, 100, 125, 200, 250, 500]

# ctcss/dcs codes
TMODES = ["", "Tone", "DTCS", "DTCS"]
TONE_NONE = 0
TONE_CTCSS = 1
TONE_DCS = 2
TONE_RDCS = 3


CTCSS_TONES = [
    67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4,
    88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9,
    114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2,
    151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3, 173.8,
    177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5,
    203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8,
    250.3, 254.1
]

# lifted from ft4.py
DTCS_CODES = [  # TODO: add negative codes
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

# flock list extended
FLOCK_LIST = ["PMR 446",
              "136-500 Mhz",
              "UNLOCK ALL",
              "DISABLE ALL"
              ]

# Scan Resum List              
SCANRESUME_LIST = ["TIMEOUT", "CARRIER", "STOP"]

# Add "CARRIER" values
for s in range(20):  # From 0 to 20s
    for ms in ["250ms", "500ms", "750ms"] if s == 0 else ["000ms", "250ms", "500ms", "750ms"]:
        SCANRESUME_LIST.append(f"CARRIER {s:02d}s:{ms} : Listen for this time until the signal disappears")

SCANRESUME_LIST.append(f"CARRIER 20s:000ms : Listen for this time until the signal disappears")

# Add "TIMEOUT" values
for m in range(5, 125, 5):  # From 5 to 120 secondes (2 minutes)
    minutes = m // 60
    seconds = m % 60
    SCANRESUME_LIST.append(f"TIMEOUT {minutes:02d}m:{seconds:02d}s : Listen for this time and resume")

# Welcome and Voice list     
WELCOME_LIST = ["Message line 1, Voltage, Sound (ALL)", "Make 2 short sounds (SOUND)", "User message line 1 and line 2 (MESSAGE)", "Battery voltage (VOLTAGE)", "NONE"]
VOICE_LIST = ["OFF", "Chinese", "English"]

# ACTIVE CHANNEL
TX_VFO_LIST = ["A", "B"]
ALARMMODE_LIST = ["SITE", "TONE"]
ROGER_LIST = ["OFF", "OURO", "KLAC", "PIU", "ICQ"]
RTE_LIST = ["OFF", "100ms", "200ms", "300ms", "400ms",
            "500ms", "600ms", "700ms", "800ms", "900ms", "1000ms"]
VOX_LIST = ["OFF", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]

MEM_SIZE =      0x00B190    # size of all memory
PROG_SIZE =     0x00A171    # size of the memory that we will write (LAST ADDRESS + 1 !!!)
MEM_BLOCK = 0x40  # VCP_RX_BUF_SIZE=128, packet=dlen+20, max dlen=64        # KA-52: VCP_RX_BUF_SIZE=128, packet=dlen+20, max dlen=64
CAL_START =     0x00B000    # calibration memory start address
SONIC_START =   0x00A158    # calibration SONIC memory start address

# fm radio supported frequencies
FMMIN = 76.0
FMMAX = 108.0

# bands supported by the UV-K5

BANDS_WIDE = {
        0: [14.0, 108.0],
        1: [108.0, 136.9999],
        2: [137.0, 173.9999],
        3: [174.0, 349.9999],
        4: [350.0, 399.9999],
        5: [400.0, 469.9999],
        6: [470.0, 2600.0] #2600 for band compatibility
        }

SCANLIST_LIST = ["OFF"] + [f"{i}" for i in range(1, MR_CHANNELS_LIST)] + ["Monitor"]

SCANLIST_SELECT_LIST = (
    [f"{i}" for i in range(1, MR_CHANNELS_LIST)]
    + ["Monitor"]
)

DTMF_CHARS = "0123456789ABCD*# "
DTMF_CHARS_ID = "0123456789ABCDabcd"
DTMF_CHARS_KILL = "0123456789ABCDabcd"
DTMF_CHARS_UPDOWN = "0123456789ABCDabcd#* "
DTMF_CODE_CHARS = "ABCD*# "
DTMF_DECODE_RESPONSE_LIST = ["DO NOTHING", "Local ringing (RING)", "Replay response (REPLY)",
                             "Local ringing + reply response (BOTH)"]

KEYACTIONS_LIST = ["NONE",
                   "FLASHLIGHT",
                   "POWER",
                   "MONITOR",
                   "SPECTRUM",
                   "SCAN",
                   "VOX",
                   "ALARM",
                   "FM RADIO",
                   "1750Hz",
                   "LOCK KEYPAD",
                   "VFO A / VFO B",
                   "VFO / MEM",
                   "MODE",
                   "BL_MIN_TMP_OFF",
                   "RX MODE",
                   "MAIN ONLY", 
                   "PTT",                  
                   "WIDE / NARROW",
                   "BACKLIGHT",
                   "MUTE",
                   "RxA",
                   "POWER HIGH",
                   "REMOVE OFFSET"
                  ]

# KA-52: только допустимые кнопки (для UI настроек)
KA52_KEYACTIONS = ["NONE", "FLASHLIGHT", "POWER", "MONITOR", "SCAN",
                   "FM RADIO", "LOCK KEYPAD", "VFO A / VFO B", "VFO / MEM",
                   "MODE", "RX MODE", "MAIN ONLY", "PTT",
                   "WIDE / NARROW", "MUTE"]


MIC_GAIN_LIST = ["+1.5dB", "+4.0dB", "+8.0dB", "+12.0dB", "+16.0dB", "+20.0dB", "+24.0dB", "+28.0dB", "+31.5dB"]

def xorarr(data: bytes):
    """the communication is obfuscated using this fine mechanism"""
    tbl = [22, 108, 20, 230, 46, 145, 13, 64, 33, 53, 213, 64, 19, 3, 233, 128]
    ret = b""
    idx = 0
    for byte in data:
        ret += bytes([byte ^ tbl[idx]])
        idx = (idx+1) % len(tbl)
    return ret


def calculate_crc16_xmodem(data: bytes):
    """
    if this crc was used for communication to AND from the radio, then it
    would be a measure to increase reliability.
    but it's only used towards the radio, so it's for further obfuscation
    """
    poly = 0x1021
    crc = 0x0
    for byte in data:
        crc = crc ^ (byte << 8)
        for _ in range(8):
            crc = crc << 1
            if crc & 0x10000:
                crc = (crc ^ poly) & 0xFFFF
    return crc & 0xFFFF


def _send_command(serport, data: bytes):
    """Send a command to UV-K5 radio"""
    LOG.debug("Sending command (unobfuscated) len=0x%4.4x:\n%s",
              len(data), util.hexprint(data))

    crc = calculate_crc16_xmodem(data)
    data2 = data + struct.pack("<H", crc)

    command = struct.pack(">HBB", 0xabcd, len(data), 0) + \
        xorarr(data2) + \
        struct.pack(">H", 0xdcba)
    if DEBUG_SHOW_OBFUSCATED_COMMANDS:
        LOG.debug("Sending command (obfuscated):\n%s", util.hexprint(command))
    try:
        result = serport.write(command)
        serport.flush()
    except Exception as e:
        raise errors.RadioError("Error writing data to radio") from e
    return result


def _receive_reply(serport, timeout=4.0):
    """Читаем пакет с фоллбэком для UART-адаптеров (PL2303/CH340).
    Скопировано из UV_AIR_TOOL receive_reply — работает с KA-52."""
    import time
    serport.timeout = timeout

    # Быстрый путь: читаем заголовок сразу
    header = serport.read(4)
    if len(header) == 4 and header[0] == 0xAB and header[1] == 0xCD:
        body_len = header[2]
        body = serport.read(body_len)
        if len(body) == body_len:
            footer = serport.read(4)
            if len(footer) == 4 and footer[2] == 0xDC and footer[3] == 0xBA:
                cmd2 = xorarr(body)
                LOG.debug("Received reply len=0x%4.4x", len(cmd2))
                return cmd2

    # Медленный путь: UART фрагментировал — собираем по кускам
    if not header:
        raise errors.RadioError("No response from radio (timeout)")

    buf = bytearray(header)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        serport.timeout = min(0.05, remaining)
        chunk = serport.read(64)
        if chunk:
            buf.extend(chunk)
        # Ищем маркер 0xAB 0xCD
        start = next((i for i in range(len(buf) - 1)
                      if buf[i] == 0xAB and buf[i+1] == 0xCD), -1)
        if start == -1:
            buf = buf[-1:] if buf else bytearray()
            continue
        if start > 0:
            buf = buf[start:]
        if len(buf) < 4:
            continue
        body_len = buf[2]
        if len(buf) < 4 + body_len + 4:
            continue
        fs = 4 + body_len
        if buf[fs+2] != 0xDC or buf[fs+3] != 0xBA:
            buf = buf[2:]
            continue
        cmd2 = xorarr(bytes(buf[4:4+body_len]))
        LOG.debug("Received reply (slow path) len=0x%4.4x", len(cmd2))
        return cmd2

    raise errors.RadioError("Header short read (UART fragmentation?)")


def _getstring(data: bytes, begin, maxlen):
    tmplen = min(maxlen+1, len(data))
    ss = [data[i] for i in range(begin, tmplen)]
    key = 0
    for key, val in enumerate(ss):
        if val < ord(' ') or val > ord('~'):
            return ''.join(chr(x) for x in ss[0:key])
    return ''


def _sayhello(serport):
    hellopacket = b"\x14\x05\x04\x00\x6a\x39\x57\x64"

    tries = 5
    while True:
        LOG.debug("Sending hello packet")
        _send_command(serport, hellopacket)
        rep = _receive_reply(serport)
        if rep:
            break
        tries -= 1
        if tries == 0:
            LOG.warning("Failed to initialise radio")
            raise errors.RadioError("Failed to initialize radio")
    if rep.startswith(b'\x18\x05'):
        raise errors.RadioError("Radio is in programming mode, "
                                "restart radio into normal mode")
    firmware = _getstring(rep, 4, 24)

    LOG.info("Found firmware: %s", firmware)
    return firmware


def _readmem(serport, offset, length):
    LOG.debug("Sending readmem offset=0x%4.4x len=0x%4.4x", offset, length)

    readmem = b"\x1b\x05\x08\x00" + \
        struct.pack("<HBB", offset, length, 0) + \
        b"\x6a\x39\x57\x64"
    _send_command(serport, readmem)
    rep = _receive_reply(serport)
    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("readmem Received data len=0x%4.4x:\n%s",
                  len(rep), util.hexprint(rep))
    return rep[8:]


def _writemem(serport, data, offset):
    LOG.debug("Sending writemem offset=0x%4.4x len=0x%4.4x",
              offset, len(data))

    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("writemem sent data offset=0x%4.4x len=0x%4.4x:\n%s",
                  offset, len(data), util.hexprint(data))

    dlen = len(data)
    writemem = b"\x1d\x05" + \
        struct.pack("<BBHBB", dlen+8, 0, offset, dlen, 1) + \
        b"\x6a\x39\x57\x64"+data

    _send_command(serport, writemem)
    rep = _receive_reply(serport)

    LOG.debug("writemem Received data: %s len=%i",
              util.hexprint(rep), len(rep))

    # KA-52: любой ответ = успех (как в UV_AIR_TOOL)
    if rep:
        return True

    LOG.warning("No response from writemem")
    raise errors.RadioError("No response to writemem")


def _resetradio(serport):
    resetpacket = b"\xdd\x05\x00\x00"
    _send_command(serport, resetpacket)


def do_download(radio):
    """download eeprom from radio"""
    serport = radio.pipe
    serport.timeout = 4.0
    status = chirp_common.Status()
    status.cur = 0
    status.max = MEM_SIZE
    status.msg = "Downloading from radio"
    radio.status_fn(status)

    eeprom = b""
    f = _sayhello(serport)
    if f:
        radio.FIRMWARE_VERSION = f
    else:
        raise errors.RadioError("Failed to initialize radio")

    addr = 0
    while addr < MEM_SIZE:
        data = _readmem(serport, addr, MEM_BLOCK)
        status.cur = addr
        radio.status_fn(status)

        if data and len(data) == MEM_BLOCK:
            eeprom += data
            addr += MEM_BLOCK
        else:
            raise errors.RadioError("Memory download incomplete")

    return memmap.MemoryMapBytes(eeprom)


def do_upload(radio):
    """upload configuration to radio eeprom"""

    serport = radio.pipe
    serport.timeout = 4.0

    status = chirp_common.Status()
    status.cur = 0
    status.msg = "Uploading to radio"
    step = 0
    radio.status_fn(status)

    f = _sayhello(serport)
    if f:
        radio.FIRMWARE_VERSION = f
    else:
        return False

    while True:
        if step == 0:
            start_addr = 0x000000
            stop_addr  = PROG_SIZE
            status.max = stop_addr - start_addr
            status.cur = 0
            status.msg = "Uploading to radio"
            radio.status_fn(status)
        elif step == 1 and radio.upload_calibration:
            start_addr = CAL_START
            stop_addr  = MEM_SIZE
            status.max = stop_addr - start_addr
            status.cur = 0
            status.msg = "Uploading calibration"
            radio.status_fn(status)
        else:
            break

        addr = start_addr
        while addr < stop_addr:
            remaining = stop_addr - addr
            chunk = MEM_BLOCK if remaining >= MEM_BLOCK else remaining
            dat = radio.get_mmap()[addr:addr + chunk]
            if not dat or len(dat) != chunk:
                raise errors.RadioError(
                    f"Memory upload incomplete at 0x{addr:06X} "
                    f"(wanted {chunk}, got {len(dat) if dat else 0})")
            _writemem(serport, dat, addr)
            status.cur = addr - start_addr
            radio.status_fn(status)
            addr += chunk

        step += 1

    status.msg = "Uploaded OK"
    radio.status_fn(status)
    _resetradio(serport)
    return True


def min_max_def(value, min_val, max_val, default):
    """returns value if in bounds or default otherwise"""
    if min_val is not None and value < min_val:
        return default
    if max_val is not None and value > max_val:
        return default
    return value


def list_def(value, lst, default=0):
    """return value if is in the list, default otherwise"""
    try:
        v = int(value)
    except (ValueError, TypeError):
        return default
    if isinstance(default, str):
        default = lst.index(default) if default in lst else 0
    if v < 0 or v >= len(lst):
        return default
    return v

@directory.register
class Ka52AirChirpRadio(chirp_common.CloneModeRadio):
    """Quansheng UV-K5"""
    VENDOR = "Quansheng"
    MODEL = "CHIRP_KA-52"
    BAUD_RATE = 38400
    NEEDS_COMPAT_SERIAL = False
    FIRMWARE_VERSION = ""

# this change to send power level chan in the calibration but under macos it give error
# bugfix calibration : put in comment next line: upload_calibration = False
    upload_calibration = False

# advanced settings too
    upload_advanced = False

    def _get_bands(self):
        bands = BANDS_WIDE
        return bands

    def _find_band(self, hz):
        mhz = hz/1000000.0
        bands = self._get_bands()
        for bnd, rng in bands.items():
            if rng[0] <= mhz <= rng[1]:
                return bnd
        return False

    def _get_vfo_channel_names(self):
        """generates VFO_CHANNEL_NAMES"""
        bands = self._get_bands()
        names = []
        for bnd, rng in bands.items():
            name = f"F{bnd + 1}({round(rng[0])}M-{round(rng[1])}M)"
            names.append(name + "A")
            names.append(name + "B")
        return names

    def _get_specials(self):
        """generates SPECIALS"""
        specials = {}
        for idx, name in enumerate(self._get_vfo_channel_names()):
            specials[name] = MR_CHANNELS_MAX + idx
        return specials

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = \
            'This is an experimental driver for the Quansheng UV-K5. ' \
            'It may harm your radio, or worse. Use at your own risk.\n\n' \
            'Before attempting to do any changes please download' \
            'the memory image from the radio with chirp ' \
            'and keep it. This can be later used to recover the ' \
            'original settings. \n\n' \
            'some details are not yet implemented'
        rp.pre_download = \
            "1. Turn radio on.\n" \
            "2. Connect cable to mic/spkr connector.\n" \
            "3. Make sure connector is firmly connected.\n" \
            "4. Click OK to download image from radio.\n\n" \
            "It may not work if you turn on the radio " \
            "with the cable already attached\n"
        rp.pre_upload = \
            "1. Turn radio on.\n" \
            "2. Connect cable to mic/spkr connector.\n" \
            "3. Make sure connector is firmly connected.\n" \
            "4. Click OK to upload the image to radio.\n\n" \
            "It may not work if you turn on the radio " \
            "with the cable already attached"
        return rp

    # Return information about this radio's features, including
    # how many memories it has, what bands it supports, etc
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.valid_dtcs_codes = DTCS_CODES
        rf.has_rx_dtcs = True
        rf.has_ctone = True
        rf.has_settings = True
        rf.has_comment = False
        rf.valid_name_length = 10
        rf.valid_power_levels = UVK5_POWER_LEVELS
        rf.valid_special_chans = self._get_vfo_channel_names()
        rf.valid_duplexes = ["", "-", "+", "off"]

        steps = STEPS.copy()
        steps.sort()
        rf.valid_tuning_steps = steps

        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone", "Tone->DTCS", "DTCS->Tone",
                                "->Tone", "->DTCS", "DTCS->", "DTCS->DTCS"]

        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_modes = ["FM", "NFM", "AM", "NAM", "DSB", "CW"]

        rf.valid_skips = [""]

        # This radio supports memories 1-250, 251-264 are the VFO memories
        rf.memory_bounds = (1, MR_CHANNELS_MAX)

        rf.valid_bands = []
        bands = self._get_bands()
        for _, rng in bands.items():
            rf.valid_bands.append(
                    (int(rng[0]*1000000), int(rng[1]*1000000)))
        return rf

    # Do a download of the radio from the serial port
    def load_mmap(self, filename):
        with open(filename, "rb") as f:
            data = bytearray(f.read())
        if len(data) < MEM_SIZE:
            raise errors.RadioError("Файл слишком мал: %d байт" % len(data))
        self._mmap = memmap.MemoryMapBytes(bytes(data[:MEM_SIZE]))
        self.process_mmap()

    @classmethod
    def match_model(cls, filedata, filename):
        if len(filedata) < MEM_SIZE:
            return False
        ver = filedata[0xA160:0xA161]
        return ver == b'V'  # KA52: V.5.

    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    # Do an upload of the radio to the serial port
    def sync_out(self):
        do_upload(self)

    # Convert the raw byte array into a memory object structure
    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    # Return a raw representation of the memory object, which
    # is very helpful for development
    def get_raw_memory(self, number):
        return repr(self._memobj.channel[number-1])

    def validate_memory(self, mem):
        # Ensure frequency and offset are integers (not strings or bitDE objects)
        # to prevent TypeError in parent class validation
        try:
            if not isinstance(mem.freq, int):
                mem.freq = int(mem.freq)
            if not isinstance(mem.offset, int):
                mem.offset = int(mem.offset)
        except (ValueError, TypeError):
            # If conversion fails, let parent handle it with proper error
            pass
        
        msgs = super().validate_memory(mem)

        if mem.duplex == "" :
            return msgs

        # find tx frequency
        if mem.duplex == '-':
            txfreq = mem.freq - mem.offset
        elif mem.duplex == '+':
            txfreq = mem.freq + mem.offset
        else:
            txfreq = mem.freq

        # find band
        band = self._find_band(txfreq)
        if band is False:
            msg = f"Transmit frequency {txfreq/1000000.0:.4f}MHz " \
                   "is not supported by this radio"
            msgs.append(chirp_common.ValidationWarning(msg))

        band = self._find_band(mem.freq)
        if band is False:
            msg = f"The frequency {mem.freq/1000000.0:%.4f}MHz " \
                   "is not supported by this radio"
            msgs.append(chirp_common.ValidationWarning(msg))

        return msgs

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

        _mem.rxcodeflag = rxmoval
        _mem.txcodeflag = txmoval
        _mem.rxcode = rxtoval
        _mem.txcode = txtoval

    def _get_tone(self, mem, _mem):
        # Convert bitDE values to int to handle chirp bitwise parser objects
        rxtype = int(_mem.rxcodeflag)
        txtype = int(_mem.txcodeflag)
        
        # Validate tone type indices before accessing TMODES list
        # TMODES has only 4 elements (indices 0-3), but 4-bit fields can have values 0-15
        # Invalid values (4-15) typically occur when EEPROM memory is uninitialized (0xFF)
        if rxtype >= len(TMODES):
            LOG.warning(f"Memory {mem.number}: Invalid rxcodeflag={rxtype} (expected 0-3), resetting to 0")
            rxtype = 0
            _mem.rxcodeflag = 0
            
        if txtype >= len(TMODES):
            LOG.warning(f"Memory {mem.number}: Invalid txcodeflag={txtype} (expected 0-3), resetting to 0")
            txtype = 0
            _mem.txcodeflag = 0
        
        rx_tmode = TMODES[rxtype]
        tx_tmode = TMODES[txtype]

        rx_tone = tx_tone = None

        if tx_tmode == "Tone":
            if _mem.txcode < len(CTCSS_TONES):
                tx_tone = CTCSS_TONES[_mem.txcode]
            else:
                tx_tone = 0
                tx_tmode = ""
        elif tx_tmode == "DTCS":
            if _mem.txcode < len(DTCS_CODES):
                tx_tone = DTCS_CODES[_mem.txcode]
            else:
                tx_tone = 0
                tx_tmode = ""

        if rx_tmode == "Tone":
            if _mem.rxcode < len(CTCSS_TONES):
                rx_tone = CTCSS_TONES[_mem.rxcode]
            else:
                rx_tone = 0
                rx_tmode = ""
        elif rx_tmode == "DTCS":
            if _mem.rxcode < len(DTCS_CODES):
                rx_tone = DTCS_CODES[_mem.rxcode]
            else:
                rx_tone = 0
                rx_tmode = ""

        tx_pol = txtype == 0x03 and "R" or "N"
        rx_pol = rxtype == 0x03 and "R" or "N"

        chirp_common.split_tone_decode(mem, (tx_tmode, tx_tone, tx_pol),
                                       (rx_tmode, rx_tone, rx_pol))

    # Extract a high-level memory object from the low-level memory map
    # This is called to populate a memory in the UI
    def get_memory(self, number):

        mem = chirp_common.Memory()

        if isinstance(number, str):
            ch_num = self._get_specials()[number]
            mem.extd_number = number
        else:
            ch_num = number - 1

        mem.number = ch_num + 1

        # Access the correct structure based on channel type
        if ch_num < MR_CHANNELS_MAX:
            # Regular memory channel (0-249)
            _mem = self._memobj.channel[ch_num]
        else:
            # VFO channel (250-263) -> vfo_channel[0-13] at 0x0fa0
            vfo_index = ch_num - MR_CHANNELS_MAX
            _mem = self._memobj.vfo_channel[vfo_index]

        is_empty = False
        # We'll consider any blank (i.e. 0MHz frequency) to be empty
        if (_mem.freq == 0xffffffff) or (_mem.freq == 0):
            is_empty = True

        # We'll also look at the channel attributes if a memory has them
        tmpscn = 0
        tmp_comp = 0
        if ch_num < MR_CHANNELS_MAX:
            _mem3 = self._memobj.ch_attr[ch_num]
            # scanlists - use new 5-bit scanlist field
            tmpscn = _mem3.scanlist
            tmp_comp = list_def(_mem3.compander, COMPANDER_LIST, 0)
        elif ch_num < MR_CHANNELS_MAX + 14:
            att_num = MR_CHANNELS_MAX + int((ch_num - MR_CHANNELS_MAX) / 2)
            _mem3 = self._memobj.ch_attr[att_num]
            tmp_comp = list_def(_mem3.compander, COMPANDER_LIST, 0)

        if is_empty:
            mem.empty = True
            # set some sane defaults:
            mem.power = UVK5_POWER_LEVELS[0]
            mem.extra = RadioSettingGroup("Extra", "extra")


            val = RadioSettingValueBoolean(False)
            rs = RadioSetting("frev", "FreqRev", val)
            mem.extra.append(rs)

            # PTT ID убран, TX запрет реализован через duplex="off"

            val = RadioSettingValueBoolean(False)
            rs = RadioSetting("dtmfdecode", "DTMF decode", val)
#            if self._memobj.BUILD_OPTIONS.ENABLE_DTMF_CALLING:
#                mem.extra.append(rs)

            val = RadioSettingValueList(COMPANDER_LIST)
            rs = RadioSetting("compander", "Compander", val)
            mem.extra.append(rs)

            val = RadioSettingValueList(SCANLIST_LIST)
            rs = RadioSetting("scanlists", "Scanlists", val)
            mem.extra.append(rs)

            # actually the step and duplex are overwritten by chirp based on
            # bandplan. they are here to document sane defaults for IARU r1
            # mem.tuning_step = 25.0
            # mem.duplex = "off"

            return mem

        if ch_num > MR_CHANNELS_MAX - 1:
            mem.name = self._get_vfo_channel_names()[ch_num-MR_CHANNELS_MAX]
            mem.immutable = ["name", "scanlists"]
        else:
            _mem2 = self._memobj.channelname[ch_num]
            for char in _mem2.name:
                if str(char) == "\xFF" or str(char) == "\x00":
                    break
                mem.name += str(char)
            mem.name = mem.name.rstrip().rstrip("\xff").rstrip().rstrip("\xff").rstrip()

        # Convert your low-level frequency to Hertz
        mem.freq = int(_mem.freq)*10
        mem.offset = int(_mem.offset)*10

        if _mem.offsetDir == FLAGS1_OFFSET_MINUS:
            mem.duplex = '-'
        elif _mem.offsetDir == FLAGS1_OFFSET_PLUS:
                mem.duplex = '+'
        else:
                mem.duplex = ''
                                
        # tone data
        self._get_tone(mem, _mem)

        # mode
        temp_modes = self.get_features().valid_modes
        # Convert modulation and bandwidth to int (they may be bitDE objects)
        modulation = int(_mem.modulation)
        bandwidth = int(_mem.bandwidth)
        temp_modul = modulation * 2 + bandwidth
        
        if temp_modul < len(temp_modes):
            _m = temp_modes[temp_modul]
            mem.mode = "USB" if _m == "DSB" else _m
        elif temp_modul == 5:  # USB with narrow setting
            mem.mode = temp_modes[4]
        elif temp_modul >= len(temp_modes):
            # Invalid modulation (corrupt data), use FM as safe default
            LOG.warning(f"Memory {mem.number}: Invalid modulation={modulation}, bandwidth={bandwidth}, "
                       f"using FM as default")
            mem.mode = "FM"  # Safe default instead of invalid string
            # Also clean up the corrupt values
            _mem.modulation = 0
            _mem.bandwidth = 0

        # tuning step
        tstep = int(_mem.step)
        if tstep < len(STEPS):
            mem.tuning_step = STEPS[tstep]
        else:
            LOG.warning(f"Memory {mem.number}: Invalid step={tstep}, using 2.5 as default")
            mem.tuning_step = 2.5

        # power
        # byte0C: freq_rev:1, bw:1, txpower:3, busyCL:1, unused:2
        # Пересобираем байт из bitfields и извлекаем txpower вручную
        _b0C_raw = (int(_mem.freq_reverse) & 1) | \
                   ((int(_mem.bandwidth) & 1) << 1) | \
                   ((int(_mem.txpower) & 7) << 2) | \
                   ((int(_mem.busyChLockout) & 1) << 5)
        txpower = (_b0C_raw >> 2) & 7
        # KA-52: 0=TX off, 1=L, 2=M, 3=H, 4=U
        _pmap = {0: UVK5_POWER_LEVELS[0], 1: UVK5_POWER_LEVELS[0],
                 2: UVK5_POWER_LEVELS[1], 3: UVK5_POWER_LEVELS[2],
                 4: UVK5_POWER_LEVELS[3]}
        mem.power = _pmap.get(txpower, UVK5_POWER_LEVELS[0])
        # We'll consider any blank (i.e. 0MHz frequency) to be empty
        if (_mem.freq == 0xffffffff) or (_mem.freq == 0):
            mem.empty = True
        else:
            mem.empty = False

        mem.extra = RadioSettingGroup("Extra", "extra")

# Frequency reverse
        val = RadioSettingValueBoolean(_mem.freq_reverse)
        rs = RadioSetting("frev", "Reverse Frequencies (R)", val)
        rs.set_doc('R: Is this needs to be reversed?')
        mem.extra.append(rs)

        mem.extra.append(RadioSetting("tx_ban", "TX Forbidden",
            RadioSettingValueBoolean(bool(_mem.tx_ban))))

# Compander
        val = RadioSettingValueList(COMPANDER_LIST, None, tmp_comp)
        rs = RadioSetting("compander", "Compander (Compnd)", val)
        rs.set_doc('Compnd: Do you want to compand on this frequency?')
        mem.extra.append(rs)

        val = RadioSettingValueList(SCANLIST_LIST, None, tmpscn)
        rs = RadioSetting("scanlists", "Scanlists (SList)", val)
        rs.set_doc('SList: Is this frequency is part of a scan list?')
        mem.extra.append(rs)

        return mem


    def set_settings(self, settings):
        _mem = self._memobj
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue

            elname = element.get_name()

            # basic settings

            # VFO_A e80 ScreenChannel_A
            if elname == "VFO_A_chn":
                _mem.ScreenChannel_A = int(element.value)
                if _mem.ScreenChannel_A < MR_CHANNELS_MAX:
                    _mem.MrChannel_A = _mem.ScreenChannel_A
                elif _mem.ScreenChannel_A < MR_CHANNELS_MAX + 7:
                    _mem.FreqChannel_A = _mem.ScreenChannel_A
                else:
                    _mem.NoaaChannel_A = _mem.ScreenChannel_A

            # VFO_B e83
            elif elname == "VFO_B_chn":
                _mem.ScreenChannel_B = int(element.value)
                if _mem.ScreenChannel_B < MR_CHANNELS_MAX:
                    _mem.MrChannel_B = _mem.ScreenChannel_B
                elif _mem.ScreenChannel_B < MR_CHANNELS_MAX + 7:
                    _mem.FreqChannel_B = _mem.ScreenChannel_B
                else:
                    _mem.NoaaChannel_B = _mem.ScreenChannel_B

            # TX_VFO  channel selected A,B
            elif elname == "TX_VFO":
                _mem.TX_VFO = int(element.value)

            # call channel
            elif elname == "call_channel":
                _mem.sl.call_channel = int(element.value)

            # squelch
            elif elname == "squelch":
                _mem.squelch = int(element.value)

            # TOT
            elif elname == "tot":
                _mem.max_talk_time = int(element.value)

            # NOAA autoscan
            elif elname == "noaa_autoscan":
                _mem.noaa_autoscan = int(element.value)

            # VOX
            elif elname == "vox":
                voxvalue = int(element.value)
                _mem.vox_switch = voxvalue > 0
                _mem.vox_level = (voxvalue - 1) if _mem.vox_switch else 0

            # mic gain
            elif elname == "mic_gain":
                _mem.mic_gain = int(element.value)

            # Channel display mode
            elif elname == "channel_display_mode":
                _mem.channel_display_mode = int(element.value)

            # RX Mode
            elif elname == "rx_mode":
                tmptxmode = int(element.value)
                tmpmainvfo = _mem.TX_VFO + 1
                _mem.crossband = tmpmainvfo * bool(tmptxmode & 0b10)
                _mem.dual_watch = tmpmainvfo * bool(tmptxmode & 0b01)

            # Battery Save
            elif elname == "battery_save":
                _mem.battery_save = int(element.value)

            # Backlight auto mode
            elif elname == "backlight_time":
                _mem.backlight_time = int(element.value)

            # Backlight min
            elif elname == "backlight_min":
                _mem.backlight_min = int(element.value)

            # Backlight max
            elif elname == "backlight_max":
                _mem.backlight_max = int(element.value)

            # Backlight TX_RX
            elif elname == "backlight_on_TX_RX":
                _mem.backlight_on_TX_RX = int(element.value)
            # AM_fix
            elif elname == "AM_fix":
                _mem.AM_fix = int(element.value)

            # mic_bar
            elif elname == "mic_bar":
                _mem.mic_bar = int(element.value)

            # Batterie txt
            elif elname == "battery_text":
                _mem.battery_text = int(element.value)

            # Tail tone elimination
            elif elname == "ste":
                _mem.ste = int(element.value)

            # VFO Open
            #elif elname == "freq_mode_allowed":
            #    _mem.freq_mode_allowed = int(element.value)

            # Current State
            elif elname == "current_state":
                _mem.current_state = int(element.value)


            # Beep control
            elif elname == "button_beep":
                _mem.button_beep = int(element.value)

            # Scan resume mode
            elif elname == "scan_resume_mode":
                _mem.scan_resume_mode = int(element.value)

            # Keypad lock
            elif elname == "key_lock":
                _mem.key_lock = int(element.value)

            # Set nav
            elif elname == "set_nav":
                if self.upload_advanced:
                    _mem.set_nav = int(element.value)

            # Auto keypad lock
            elif elname == "auto_keypad_lock":
                _mem.auto_keypad_lock = int(element.value)

            # Power on display mode
            elif elname == "welcome_mode":
                _mem.power_on_dispmode = int(element.value)

            # Keypad Tone
            elif elname == "voice":
                _mem.voice = int(element.value)

            elif elname.startswith("dbm_corr_"):
                if self.upload_advanced:
                    i = int(elname.split("_")[-1])
                    _mem.dbm_corr[i] = int(element.value)

#            elif elname == "password":
#                if element.value.get_value() is None or element.value == "":
#                    _mem.password = 0xFFFFFFFF
#                else:
#                    _mem.password = int(element.value)

            # Alarm mode
            elif elname == "alarm_mode":
                _mem.alarm_mode = int(element.value)

            # Reminding of end of talk
            elif elname == "roger_beep":
                _mem.roger_beep = int(element.value)

            # Repeater tail tone elimination
            elif elname == "rp_ste":
                _mem.rp_ste = int(element.value)

            # Logo string 1
            elif elname == "logo1":
                bts = str(element.value).rstrip("\x20\xff\x00")+"\x00"*12
                _mem.logo_line1 = bts[0:12]+"\x00\xff\xff\xff"

            # Logo string 2
            elif elname == "logo2":
                bts = str(element.value).rstrip("\x20\xff\x00")+"\x00"*12
                _mem.logo_line2 = bts[0:12]+"\x00\xff\xff\xff"

            # unlock settings

            # FLOCK
            elif elname == "int_flock":
                _mem.int_flock = int(element.value)

            # KILLED
            elif elname == "int_KILLED":
                _mem.int_KILLED = int(element.value)

            # SCREN
            elif elname == "int_scren":
                _mem.int_scren = int(element.value)

            # battery type
            elif elname == "Battery_type":
                _mem.Battery_type = int(element.value)

            # set low_power SONIC
            elif elname == "set_pwr":
                _mem.set_pwr = int(element.value)

            # set ptt SONIC
            elif elname == "set_ptt":
                _mem.set_ptt = int(element.value)

            # set tot SONIC
            elif elname == "set_tot":
                _mem.set_tot = int(element.value)

            # set eot SONIC
            elif elname == "set_eot":
                _mem.set_eot = int(element.value)

            # set_contrast SONIC
            elif elname == "set_contrast":
                _mem.set_contrast = int(element.value)

            # set inv SONIC
            elif elname == "set_inv":
                _mem.set_inv = int(element.value)

            # set lck SONIC
            elif elname == "set_lck":
                _mem.set_lck = int(element.value)

            # set met SONIC
            elif elname == "set_met":
                _mem.set_met = int(element.value)

            # set gui SONIC
            elif elname == "set_gui":
                _mem.set_gui = int(element.value)
                               
            # set tmr SONIC
            elif elname == "set_tmr":
                _mem.set_tmr = int(element.value)

            # set off SONIC
            elif elname == "set_off_tmr":
                _mem.set_off_tmr = int(element.value)

            # set nfm SONIC
            elif elname == "set_nfm":
                _mem.set_nfm = int(element.value)

            # set rxa SONIC
            elif elname == "set_rxa":
                _mem.set_rxa = int(element.value)

            # set key SONIC
            elif elname == "set_key":
                _mem.set_key = int(element.value)

             # set menu lock SONIC
            elif elname == "set_menu_lock":
                _mem.set_menu_lock = int(element.value)

            # fm radio
            for i in range(1, FM_CHANNELS_MAX + 1):
                freqname = "FM_" + str(i)
                if elname == freqname:
                    val = str(element.value).strip()
                    try:
                        val2 = int(float(val)*10)
                    except Exception:
                        val2 = 0xffff

                    if val2 < FMMIN*10 or val2 > FMMAX*10:
                        val2 = 0xffff
#                        raise errors.InvalidValueError(
#                                "FM radio frequency should be a value "
#                                "in the range %.1f - %.1f" % (FMMIN , FMMAX))
                    _mem.fmfreq[i-1] = val2

            # dtmf settings
            if elname == "dtmf_side_tone":
                _mem.dtmf.side_tone = int(element.value)

            elif elname == "dtmf_separate_code":
                _mem.dtmf.separate_code = str(element.value)

            elif elname == "dtmf_group_call_code":
                _mem.dtmf.group_call_code = element.value

            elif elname == "dtmf_decode_response":
                _mem.dtmf.decode_response = int(element.value)

            elif elname == "dtmf_auto_reset_time":
                _mem.dtmf.auto_reset_time = int(element.value)

            elif elname == "dtmf_preload_time":
                _mem.dtmf.preload_time = int(int(element.value)/10)

            elif elname == "dtmf_first_code_persist_time":
                _mem.dtmf.first_code_persist_time = int(int(element.value)/10)

            elif elname == "dtmf_hash_persist_time":
                _mem.dtmf.hash_persist_time = int(int(element.value)/10)

            elif elname == "dtmf_code_persist_time":
                _mem.dtmf.code_persist_time = \
                        int(int(element.value)/10)

            elif elname == "dtmf_code_interval_time":
                _mem.dtmf.code_interval_time = \
                        int(int(element.value)/10)

            elif elname == "dtmf_permit_remote_kill":
                _mem.dtmf.permit_remote_kill = \
                        int(element.value)

            elif elname == "dtmf_dtmf_local_code":
                k = str(element.value).rstrip("\x20\xff\x00") + "\x00"*3
                _mem.dtmf.local_code = k[0:3]

            elif elname == "dtmf_dtmf_up_code":
                k = str(element.value).strip("\x20\xff\x00") + "\x00"*16
                _mem.dtmf.up_code = k[0:16]

            elif elname == "dtmf_dtmf_down_code":
                k = str(element.value).rstrip("\x20\xff\x00") + "\x00"*16
                _mem.dtmf.down_code = k[0:16]

            elif elname == "dtmf_kill_code":
                k = str(element.value).strip("\x20\xff\x00") + "\x00"*5
                _mem.dtmf.kill_code = k[0:5]

            elif elname == "dtmf_revive_code":
                k = str(element.value).strip("\x20\xff\x00") + "\x00"*5
                _mem.dtmf.revive_code = k[0:5]

            elif elname == "live_DTMF_decoder":
                _mem.live_DTMF_decoder = int(element.value)

            # scanlist stuff
            if elname == "slDef":
                _mem.sl.slDef = int(element.value) + 1

            elif elname == "slPriorEnab":
                _mem.sl.slPriorEnab = int(element.value)

            elif elname == "slPriorCh1":
                _mem.sl.slPriorCh1 = int(element.value)

            elif elname == "slPriorCh2":
                _mem.sl.slPriorCh2 = int(element.value)

            elif elname.startswith("listname"):
                idx = int(elname.replace("listname", ""))
                if 0 <= idx < (MR_CHANNELS_LIST - 1):
                    val_str = str(element.value).strip()
                    
                    if val_str:  # Si non vide
                        val_bytes = val_str.encode('ascii', 'ignore')[:3]
                        # Pad avec 0xFF comme le firmware
                        val_bytes = val_bytes + b'\xFF' * (4 - len(val_bytes))
                    else:  # Si vide, remplir avec 0xFF
                        val_bytes = b'\xFF' * 4
                        
                    _mem.listname[idx].name = val_bytes

            # Shortcuts

            if elname == "key1_shortpress_action":
                _mem.key1_shortpress_action = KEYACTIONS_LIST.index(element.value)

            elif elname == "key1_longpress_action":
                _mem.key1_longpress_action = KEYACTIONS_LIST.index(element.value)

            elif elname == "key2_shortpress_action":
                _mem.key2_shortpress_action = KEYACTIONS_LIST.index(element.value)

            elif elname == "key2_longpress_action":
                _mem.key2_longpress_action = KEYACTIONS_LIST.index(element.value)

            elif elname == "keyM_longpress_action":
                _mem.keyM_longpress_action = KEYACTIONS_LIST.index(element.value)

# this change to send power level chan in the calibration but under macos it give error
# bugfix calibration : remove the comment on next 2 line:
#            elif elname == "upload_calibration":
#                self._upload_calibration = bool(element.value)

            elif element.changed() and elname.startswith("_mem.cal."):
                exec(elname + " = element.value.get_value()")

    def get_settings(self):
        _mem = self._memobj

        basic       = RadioSettingGroup("basic",       "Basic Settings")
        advanced    = RadioSettingGroup("advanced",    "Advanced Settings")
        keya        = RadioSettingGroup("keya",        "Programmable Keys")
        calibration = RadioSettingGroup("calibration", "Calibration")
        top = RadioSettings()
        top.append(basic); top.append(advanced)
        top.append(keya); top.append(calibration)

        def _i(v):
            try: return int(v)
            except: return 0

        def li(g, a, lb, lst):
            idx = list_def(getattr(_mem, a), lst)
            g.append(RadioSetting(a, lb,
                RadioSettingValueList(lst, None, idx)))

        def bi(g, a, lb):
            g.append(RadioSetting(a, lb,
                RadioSettingValueBoolean(bool(_i(getattr(_mem, a))))))

        def ii(g, a, lb, mn, mx):
            val = max(mn, min(mx, _i(getattr(_mem, a))))
            g.append(RadioSetting(a, lb,
                RadioSettingValueInteger(mn, mx, val)))

        li(basic, "TX_VFO",              "Active VFO",      TX_VFO_LIST)
        ii(basic, "squelch",             "Squelch",          0, 9)
        li(basic, "channel_display_mode","Channel Display",  CHANNELDISP_LIST)
        li(basic, "battery_save",        "Battery Save",     BATSAVE_LIST)
        li(basic, "backlight_time",      "Backlight Time",   BACKLIGHT_LIST)
        li(basic, "backlight_on_TX_RX",  "Backlight TX/RX",  BACKLIGHT_TX_RX_LIST)
        li(basic, "battery_text",        "Battery Display",  BAT_TXT_LIST)
        li(basic, "roger_beep",          "Roger Tone",       ROGER_LIST)
        li(basic, "scan_resume_mode",    "Scan Resume",      SCANRESUME_LIST)
        li(basic, "int_flock",           "TX Freq Lock",     FLOCK_LIST)

        for a, lb in [("logo_line1","Welcome Line 1"),("logo_line2","Welcome Line 2")]:
            try: txt = str(getattr(_mem, a)).rstrip("\xff\x00 ").strip()[:16]
            except: txt = ""
            basic.append(RadioSetting(a, lb, RadioSettingValueString(0, 16, txt)))

        bi(advanced, "AM_fix",    "AM Fix")
        bi(advanced, "mic_bar",   "MIC Bar")
        ii(advanced, "mic_gain",  "MIC Gain",  0, 4)
        bi(advanced, "vox_switch","VOX")
        ii(advanced, "vox_level", "VOX Level", 0, 9)

        for attr, label in [
            ("key1_shortpress_action","Side Key 1 Short"),
            ("key1_longpress_action", "Side Key 1 Long"),
            ("key2_shortpress_action","Side Key 2 Short"),
            ("key2_longpress_action", "Side Key 2 Long"),
            ("keyM_longpress_action", "Menu Key Long"),
        ]:
            idx = list_def(getattr(_mem, attr), KEYACTIONS_LIST)
            keya.append(RadioSetting(attr, label,
                RadioSettingValueList(KEYACTIONS_LIST, None, idx)))

        v = RadioSettingValueString(21, 21, "Edit with UV AIR TOOL")
        v.set_mutable(False)
        calibration.append(RadioSetting("cal", "Calibration", v))

        return top

    def set_memory(self, memory):
        """
        Store details about a high-level memory to the memory map
        This is called when a user edits a memory in the UI
        """
        number = memory.number-1
        att_num = number if number < MR_CHANNELS_MAX else MR_CHANNELS_MAX + int((number - MR_CHANNELS_MAX) / 2)

        # Get a low-level memory object mapped to the image
        # Access the correct structure based on channel type
        if number < MR_CHANNELS_MAX:
            # Regular memory channel (0-249)
            _mem_chan = self._memobj.channel[number]
        else:
            # VFO channel (250-263) -> vfo_channel[0-13] at 0x0fa0
            vfo_index = number - MR_CHANNELS_MAX
            _mem_chan = self._memobj.vfo_channel[vfo_index]
        
        _mem_attr = self._memobj.ch_attr[att_num]

        # Initialize scanlist fields (new 5-bit field and old 1-bit flags)
        _mem_attr.scanlist = 0
        _mem_attr.compander = 0

        # empty memory
        if memory.empty:
            _mem_chan.set_raw(b"\xFF" * 16)

            if number < MR_CHANNELS_MAX:
                _mem_chname = self._memobj.channelname[number]
                _mem_chname.set_raw(b"\x20" * 16)

                # deleted marker: 0xFFFF
                _mem_attr = self._memobj.ch_attr[number]   # ou att_num, ici c'est pareil pour MR
                _mem_attr.set_raw(b"\xFF\xFF")
            
            return memory

        # find band
        band = self._find_band(memory.freq)

        # mode
        tmp_mode = self.get_features().valid_modes.index(memory.mode)
        _mem_chan.modulation = tmp_mode / 2
        _mem_chan.bandwidth = tmp_mode % 2
        if memory.mode == "USB":
            _mem_chan.bandwidth = 1  # narrow

        # frequency/offset
        _mem_chan.freq = memory.freq/10
        _mem_chan.offset = memory.offset/10

        if memory.duplex == "":
            _mem_chan.offsetDir = FLAGS1_OFFSET_NONE
        elif memory.duplex == '-':
            _mem_chan.offsetDir = FLAGS1_OFFSET_MINUS
        elif memory.duplex == '+':
            _mem_chan.offsetDir = FLAGS1_OFFSET_PLUS

        # set band

#        _mem_attr.is_free = 0
        _mem_attr.band = band

        # channels >200 are the 14 VFO chanells and don't have names
        if number < MR_CHANNELS_MAX:
            _mem_chname = self._memobj.channelname[number]
            tag = memory.name.ljust(10) + "\x00"*6
            _mem_chname.name = tag  # Store the alpha tag

        # tone data
        self._set_tone(memory, _mem_chan)

        # step
        _mem_chan.step = STEPS.index(memory.tuning_step)

        # KA-52 tx power: 0=off,1=L,2=M,3=H,4=U
        if str(memory.power) == str(UVK5_POWER_LEVELS[3]):
            _mem_chan.txpower = POWER_ULTRA
        elif str(memory.power) == str(UVK5_POWER_LEVELS[2]):
            _mem_chan.txpower = POWER_HIGH
        elif str(memory.power) == str(UVK5_POWER_LEVELS[1]):
            _mem_chan.txpower = POWER_MEDIUM
        else:
            _mem_chan.txpower = POWER_LOW
        # -------- EXTRA SETTINGS

        def get_setting(name, def_val):
            if name in memory.extra:
                return int(memory.extra[name].value)
            return def_val

        _mem_chan.freq_reverse = get_setting("frev", False)
        _mem_chan.tx_ban        = get_setting("tx_ban", False)
        _mem_chan.dtmf_decode = get_setting("dtmfdecode", False)
        _mem_attr.compander = get_setting("compander", 0)
        if number < MR_CHANNELS_MAX:
            tmp_val = get_setting("scanlists", 0)
            _mem_attr.scanlist = tmp_val

        return memory