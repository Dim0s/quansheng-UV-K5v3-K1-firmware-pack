# Quansheng UV-K5 driver (c) 2023 Jacek Lipkowski <sq5bpf@lipkowski.org>
# Adapted for EGZUMER by EGZUMER and JOC2; adapted for F4HWN by JOC2 and F4HWN.
# Based on template.py, Copyright 2012 Dan Smith <dsmith@danplanet.com>.
# Further adapted for Falcon firmware.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See GPL-2.0.txt or <https://www.gnu.org/licenses/> for the full license.

# Falcon CHIRP driver — navigation support added in 2.0.1-OA.
# Load this file as a CHIRP custom module, then select Quansheng / UV-K1 / UV-K5 V3 Falcon v3.0.0.
# Download a fresh image from the target radio before making edits.
# Battery options follow the downloaded firmware version; EEPROM IDs stay stable.
# Memories 4067..4096 are bands: Frequency = FSTART, Band stop (MHz) = FSTOP.
# List names: 25 x 6 characters, in the existing 10-byte EEPROM records.
# Application slots and firmware images are not uploaded by this driver.

import webbrowser
import os

import struct
from decimal import Decimal, InvalidOperation
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

DRIVER_VERSION = "Falcon driver: 2026/10/25 v3.0.0"

VALEUR_COMPILER = "ENABLE"

MEM_FORMAT = """
// -------------------- CHANNEL_LOCATION

#seekto 0x122000;
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
  u8 __UNUSED01:1,
  txLock:1,
  busyChLockout:1,
  txpower:3,
  bandwidth:1,
  freq_reverse:1;

  // 0x0D
  u8 __UNUSED02:4,
  dtmf_pttid:3,
  dtmf_decode:1;

  // 0x0E
  u8 step;
  u8 __UNUSED03;

} channel[4096];

// --------------------

#seekto 0x132000;
struct {
char name[16];
} channelname[4096];


// --------------------

#seekto 0x142000;
struct {
  u8 __UNUSED04:3,
     compander:2,
     band:3;
  u8 scanlist;
} ch_attr[4103];

// MR_CHANNELS_LIST

#seekto 0x008900;
struct {
    char name[6];
    u8 reserved[4];
} listname[25];

// FM_RADIOS_LIST

#seekto 0x008AF4;
struct {
  ul16 freq;
  char name[15];
} fmradiolist[50];
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
u8 audio_am:4,
   set_rxa:4;
u8 squelch;
u8 max_talk_time;
u8 scrambler;
u8 __UNUSED09:1,
   set_nav:1,
   set_key:4,
   set_menu_lock:1,
   key_lock:1;
u8 __UNUSED_A005:6,
   flashlight_rx:1,
   vox_switch:1;
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
u8 __UNUSED10:2,
   power_on_screen:3,
   vfo_rx_on:1,
   set_nfm:1,
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
ul16 fmfreq[9];

// --------------------

#seekto 0x00A0A8;
u8 keyM_longpress_action:7,
   button_beep:1;

u8 key1_shortpress_action;
u8 key1_longpress_action;
u8 key2_shortpress_action;
u8 key2_longpress_action;
u8 reserved_0ea5;    
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
u8 auto_roger;

// --------------------

#seekto 0x00A0C8;
char logo_line1[16];
char logo_line2[16];

// --------------------

#seekto 0x00A0E8;
u8 reserved_a0e8[72];

#seekto 0x00A130;

struct {
    u8 slPriorEnab:1,
       slDef:7;
        
    ul16 slPriorCh1;
    ul16 slPriorCh2;
    ul16 call_channel;

    u8 __UNUSED10;
} sl;

// --------------------

#seekto 0x00A150;
u8 int_flock;
u8 meter_marker;
u8 int_KILLED;
u8 int_200tx_unsused;
u8 int_500tx_unsused;
u8 int_350en;
u8 int_scren;

u8  backlight_on_TX_RX:2,
    meter_classic:1,
    mic_bar:1,
    battery_text:2,
    led_mode:2;

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

u8 set_off_tmr:5,
   button_invert:1,
   reserved_timer:1,
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

#seekto 0x00C025;
u8 listen_bw_fm;
#seekto 0x00C030;
u8 listen_bw_am;
u8 listen_bw_am_tag;
// TEST4/5 scan thresholds and shared hysteresis; existing profile offsets stay fixed.
#seekto 0x00C032;
u8 scan_sql_tag;
u8 vfo_scan_sql;
u8 spectrum_sql;
#seekto 0x00C049;
u8 scan_sql_hys;

#seekto 0x00A170;
struct { ul16 slPriorCh3; ul16 slPriorCh4; } ow;

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
FM_CHANNELS_MAX = 9  # Direct keypad presets; both firmware branches use these addresses.
FM_RADIOS_MAX = 50
FM_RADIOS_RECORD = 17   # ul16 freq + char name[15]

MR_CHANNELS_MAX = 4096 # CHANNEL_LOCATION
MR_CHANNELS_LIST = 26  # 25 named lists plus the existing index convention
BAND_FIRST = 4067
BAND_NAME_LEN = 8
LIST_NAME_LEN = 6
MONITOR_ID = 51

# flags1
FLAGS1_OFFSET_NONE = 0b00
FLAGS1_OFFSET_MINUS = 0b10
FLAGS1_OFFSET_PLUS = 0b01


POWER_HIGH =    0b010
POWER_MEDIUM =  0b001
POWER_LOW =     0b000

# SET_LOW_POWER SONIC
SET_LOW_LIST = ["L1 (<20mW)", "L2 (125mW)", "L3 (250mW)", "L4 (500mW)", "L5 (1W)", "MID", "HIGH"]
POWER_VALUES = [0, 3, 4, 5, 6, 1, 2]

# SET_TOT and SET_EOT SONIC
SET_TOT_EOT_LIST = ["OFF", "SOUND", "VISUAL", "ALL"]

# SET_OFF_ON SONIC
SET_OFF_ON_LIST = ["OFF", "ON"]

# SET_lck SONIC
SET_LCK_LIST = ["KEYS", "KEYS+PTT"]

# power          
UVK5_POWER_LEVELS = [chirp_common.PowerLevel(name) for name in
                     ["L1", "L2", "L3", "L4", "L5", "MID", "HIGH"]]

# compander
COMPANDER_LIST = ["OFF", "TX", "RX", "TX/RX"]

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
BATTYPE_LIST = ["1400 mAh UV-K1", "2500 mAh UV-K1", "1600 mAh UV-K5",
                "2200 mAh UV-K5", "3500 mAh UV-K5"]
BATTYPE_VALUES = [5, 4, 0, 1, 2]
LEGACY_BATTYPE_LIST = ["1400 mAh UV-K1 Falcon"] + BATTYPE_LIST
LEGACY_BATTYPE_VALUES = [3] + BATTYPE_VALUES
AUTO_ROGER_LIST = ["OFF", "10sec", "30sec", "2mn", "5mn", "10mn", "30mn"]
LISTEN_BW_FM_LIST = ["WIDE", "NARROW", "NARROWER"]
LISTEN_BW_AM_LIST = ["WIDE", "NARROW"]
# bat txt
BAT_TXT_LIST = ["NONE", "VOLTAGE", "PERCENT", "VOLTAGE + PERCENT"]
LED_LIST = ["On", "SOS", "Beacon", "Off"]
LED_VALUES = [1, 2, 3, 0]  # Firmware storage: Off=0, On=1, SOS=2, Beacon=3.
RX_MODE_LIST = ["SINGLE", "DUAL", "FULL (Triple / Quad watch)", "ML WATCH"]
RX_MODE_VALUES = [0, 1, 4, 12]
METER_STYLE_LIST = ["FALCON", "CLASSIC"]
SET_RXA_AM_LIST = ["SHARP", "STOCK", "OPEN"]
TONE_1750_LIST = ["1750", "OFF"]
TONE_1750_VALUES = [1, 0]

def sonic_rx_config(value):
    """Mirror the firmware fallback for invalid legacy RX/Tone bytes."""
    value = int(value)
    return value if value & 0xF0 == 0 else 0

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
FLOCK_LIST = ["UNLOCK ALL", "DISABLE ALL", "VHF-UHF", "PMR + LPD"]

# Welcome and Voice list     
WELCOME_LIST = ["MESSAGE", "LOGO", "NONE"]
PONSCR_LIST = ["NORMAL", "SPECTRE LIST", "SPECTRE BAND", "SPECTRE RANGE", "SENTINEL", "FM RADIO"]
VOICE_LIST = ["OFF", "Chinese", "English"]

# ACTIVE CHANNEL
TX_VFO_LIST = ["A", "B"]
ALARMMODE_LIST = ["SITE", "TONE"]
ROGER_LIST = ["OFF", "STARWARS", "R2D2", "FALCON", "ROGGER", "OURO"]
ROGER_VALUES = [0, 10, 3, 9, 4, 6]
RTE_LIST = ["OFF", "100ms", "200ms", "300ms", "400ms",
            "500ms", "600ms", "700ms", "800ms", "900ms", "1000ms"]

MEM_SIZE =      0x144000    # size of all memory CHANNEL_LOCATION
PROG_SIZE =     0x144001    # size of the memory that we will write (LAST ADDRESS + 1 !!!)
MEM_BLOCK =     0x80        # largest block of memory that we can reliably write
CAL_START =     0x00B000    # calibration memory start address

# fm radio supported frequencies
FMMIN = 76.0
FMMAX = 108.0
FM_MEMORY_MIN = 87.5  # Sonic v1.0 fixed FM band 0
FM_MEMORY_MAX = 108.0

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

# Numeric ids must match ACTION_OPT_t in Sonic v1.0/settings.h.
# Reserved/no-op ids stay in this table so later actions never shift.
KEYACTIONS_LIST = ["NONE", "FLASHLIGHT", "POWER", "MONITOR", "SCAN",
                   "ALARM", "FM RADIO", "1750", "LOCK KEYPAD", "FREQ / MEM",
                   "MODE", "BL_MIN_TMP_OFF", "PTT", "WIDE / NARROW",
                   "BACKLIGHT", "MUTE", "RxA", "ScanLst Up", "ScanLst Dw",
                   "RXMOD", "SCAN ALL", "RESERVED 21", "RESERVED 22", "PTT 2", "VFO A/B",
                   "HISTORY", "VOX"]
# Same assignable actions as the v1.0 radio menu; legacy functional actions
# (1750, BACKLIGHT and RxA) are retained when already assigned.
KEYACTIONS_SUPPORTED = [0, 1, 2, 3, 4, 6, 8, 9, 10, 12, 13, 17, 18, 19, 20, 23, 24, 25, 26]
KEYACTIONS_LEGACY = [7, 14, 16]
SCAN_SQL_LIST = [f"{v} dbm" for v in range(-130, -52)]
SCAN_HYS_LIST = [f"{v} dbm" for v in range(9)]


VOX_LIST = ["OFF"] + [f"{i}" for i in range(1, 11)]
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
    except Exception as e:
        raise errors.RadioError("Error writing data to radio") from e
    return result


def _receive_reply(serport):
    header = serport.read(4)
    if len(header) != 4:
        LOG.warning("Header short read: [%s] len=%i",
                    util.hexprint(header), len(header))
        raise errors.RadioError("Header short read")
    if header[0] != 0xAB or header[1] != 0xCD or header[3] != 0x00:
        LOG.warning("Bad response header: %s len=%i",
                    util.hexprint(header), len(header))
        raise errors.RadioError("Bad response header")

    cmd = serport.read(int(header[2]))
    if len(cmd) != int(header[2]):
        LOG.warning("Body short read: [%s] len=%i",
                    util.hexprint(cmd), len(cmd))
        raise errors.RadioError("Command body short read")

    footer = serport.read(4)

    if len(footer) != 4:
        LOG.warning("Footer short read: [%s] len=%i",
                    util.hexprint(footer), len(footer))
        raise errors.RadioError("Footer short read")

    if footer[2] != 0xDC or footer[3] != 0xBA:
        LOG.debug("Reply before bad response footer (obfuscated)"
                  "len=0x%4.4x:\n%s", len(cmd), util.hexprint(cmd))
        LOG.warning("Bad response footer: %s len=%i",
                    util.hexprint(footer), len(footer))
        raise errors.RadioError("Bad response footer")

    if DEBUG_SHOW_OBFUSCATED_COMMANDS:
        LOG.debug("Received reply (obfuscated) len=0x%4.4x:\n%s",
                  len(cmd), util.hexprint(cmd))

    cmd2 = xorarr(cmd)

    LOG.debug("Received reply (unobfuscated) len=0x%4.4x:\n%s",
              len(cmd2), util.hexprint(cmd2))

    return cmd2


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
    # The v7 prefix selects 64 Apps in UVStudio; present the Falcon release here.
    if firmware == "FALCON 7.0/3.0":
        firmware = "FALCON 3.0.0"
    elif firmware in ("FALCON 6.3/2.3", "FALCON 6.2/2.2"):
        marker = _readmem(serport, 0xA160, 16).split(b"\0", 1)[0]
        release = marker.removesuffix(b".D1").decode("ascii", errors="replace")
        if release and all(c in "0123456789.v" for c in release):
            firmware = "FALCON " + release


    if not firmware.upper().startswith(("FALCON ", "SONIC ")):
        raise errors.RadioError("This driver requires Falcon/Sonic firmware. "
                                "Select the F4HWN driver for F4HWN firmware.")

    LOG.info("Found firmware: %s", firmware)
    return firmware

def _readmem(serport, offset, length):
    LOG.debug("Sending readmem offset=0x%8.8x len=0x%4.4x", offset, length)

    # Layout attendu par CMD_051B_t (avec padding d'alignement) :
    # [0-1] ID=0x051B  [2-3] Size  [4-7] Offset32  [8] Size  [9] Pad  [10-11] padding  [12-15] Timestamp
    readmem = b"\x1b\x05\x0c\x00" + \
        struct.pack("<IBB", offset, length, 0) + \
        b"\x00\x00" + \
        b"\x6a\x39\x57\x64"       # Timestamp = 0x6457396a (session ID du hello)
    _send_command(serport, readmem)
    rep = _receive_reply(serport)
    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("readmem Received data len=0x%4.4x:\n%s",
                  len(rep), util.hexprint(rep))
    # Réponse REPLY_051B : Header(4) + Offset(4) + Size(1) + Pad(1) → données à rep[10:]
    return rep[10:]

def _writemem(serport, data, offset):
    LOG.debug("Sending writemem offset=0x%8.8x len=0x%4.4x",
              offset, len(data))

    if DEBUG_SHOW_MEMORY_ACTIONS:
        LOG.debug("writemem sent data offset=0x%8.8x len=0x%4.4x:\n%s",
                  offset, len(data), util.hexprint(data))

    dlen = len(data)
    # CMD_051D_t : [0-1] ID  [2-3] Size  [4-7] Offset32  [8] Size  [9] AllowPwd
    #              [10-11] padding  [12-15] Timestamp  [16+] Data
    writemem = b"\x1d\x05" + \
        struct.pack("<H", dlen + 12) + \
        struct.pack("<IBB", offset, dlen, 1) + \
        b"\x00\x00" + \
        b"\x6a\x39\x57\x64" + data

    _send_command(serport, writemem)
    rep = _receive_reply(serport)

    LOG.debug("writemem Received data: %s len=%i",
              util.hexprint(rep), len(rep))

    # REPLY_051D : Offset est uint16_t dans votre firmware → seulement rep[4] et rep[5]
    if (len(rep) >= 6 and
       rep[0] == 0x1e and
       rep[4] == (offset & 0xff) and
       rep[5] == ((offset >> 8) & 0xff)):
        return True

    LOG.warning("Bad data from writemem")
    raise errors.RadioError("Bad response to writemem")

def _resetradio(serport):
    resetpacket = b"\xdd\x05\x00\x00"
    _send_command(serport, resetpacket)


def do_download(radio):
    """download only necessary blocks from radio eeprom"""
    serport = radio.pipe
    serport.timeout = 4.0
    status = chirp_common.Status()
    status.msg = "Downloading from radio"

    # Alloue exactement la taille attendue définie par MEM_SIZE
    eeprom = bytearray(b"\xFF" * radio.MEM_SIZE)

    f = _sayhello(serport)
    if f:
        radio.FIRMWARE_VERSION = f
    else:
        raise errors.RadioError("Failed to initialize radio")

    # Plages de mémoire à lire CHANNEL_LOCATION
    blocks_to_read = [
        (0x008900, 0x00A174),
        (0x00B000, 0x00B200),
        (0x00C020, 0x00C088),
        (0x122000, 0x122000 + (MR_CHANNELS_MAX * 16)),
        (0x132000, 0x132000 + (MR_CHANNELS_MAX * 16)),
        (0x142000, 0x142000 + (MR_CHANNELS_MAX * 2)),
    ]

    total_bytes = sum(stop - start for start, stop in blocks_to_read)
    status.max = total_bytes
    bytes_read = 0

    for start_addr, stop_addr in blocks_to_read:
        addr = start_addr
        while addr < stop_addr:
            chunk = min(MEM_BLOCK, stop_addr - addr)
            data = _readmem(serport, addr, chunk)

            if data and len(data) == chunk:
                eeprom[addr:addr + chunk] = data
                addr += chunk
                bytes_read += chunk
                status.cur = bytes_read
                radio.status_fn(status)
            else:
                raise errors.RadioError(f"Memory download incomplete at 0x{addr:06X}")

    return memmap.MemoryMapBytes(bytes(eeprom))


def do_upload(radio):
    """upload configuration to radio eeprom skipping unused ranges"""
    serport = radio.pipe
    serport.timeout = 4.0

    status = chirp_common.Status()
    status.msg = "Uploading to radio"
    radio.status_fn(status)

    f = _sayhello(serport)
    if not f:
        return False
    radio.FIRMWARE_VERSION = f

    # Plages de mémoire à écrire en mode standard CHANNEL_LOCATION
    blocks_to_write = [
        (0x008900, 0x00A174),  # Configs & VFOs
        (0x122000, 0x122000 + (MR_CHANNELS_MAX * 16)),  # Canaux
        (0x132000, 0x132000 + (MR_CHANNELS_MAX * 16)),  # Noms
        (0x142000, 0x142000 + (MR_CHANNELS_MAX * 2)),   # Attributs
    ]

    if radio.upload_listenbw:
        blocks_to_write.extend([(0x00C025, 0x00C026), (0x00C030, 0x00C032)])

    if radio.upload_sqlscn:
        # Retain the radio's live spectrum threshold even when the local image
        # predates the current spectrum setting. Only SQLSCN is being edited.
        scan_sql = _readmem(serport, 0x00C032, 3)
        if not scan_sql or len(scan_sql) != 3:
            raise errors.RadioError("Unable to preserve spectrum squelch")
        radio._memobj.spectrum_sql = scan_sql[2] if scan_sql[0] == 0xD2 and scan_sql[2] <= 77 else 30
        blocks_to_write.append((0x00C032, 0x00C035))
    if radio.upload_sqlhys:
        blocks_to_write.append((0x00C049, 0x00C04A))

    # Ajout du bloc de calibration si l'option est cochée
    if radio.upload_calibration:
        blocks_to_write.append((CAL_START, 0x00B200))
    elif radio.upload_volume:
        # SetVol uses this one existing byte; preserve crystal and DAC calibration.
        blocks_to_write.append((0x00B18E, 0x00B18F))

    total_bytes = sum(stop - start for start, stop in blocks_to_write)
    status.max = total_bytes
    bytes_written = 0

    mmap = radio.get_mmap()
    # A profile image must not re-arm the firmware's first-install defaults.
    defaults_marker = _readmem(serport, 0xA160, 16)

    for start_addr, stop_addr in blocks_to_write:
        addr = start_addr
        while addr < stop_addr:
            chunk = min(MEM_BLOCK, stop_addr - addr)
            dat = mmap[addr:addr + chunk]

            if not dat or len(dat) != chunk:
                raise errors.RadioError(f"Memory upload incomplete at 0x{addr:06X}")

            if addr < 0xA170 and addr + chunk > 0xA160 and len(defaults_marker) == 16 and b".D1" in defaults_marker:
                dat = bytearray(dat)
                lo, hi = max(addr, 0xA160), min(addr + chunk, 0xA170)
                dat[lo - addr:hi - addr] = defaults_marker[lo - 0xA160:hi - 0xA160]
                dat = bytes(dat)
            _writemem(serport, dat, addr)

            addr += chunk
            bytes_written += chunk
            status.cur = bytes_written
            radio.status_fn(status)

    status.msg = "Uploaded OK"
    radio.status_fn(status)

    _resetradio(serport)
    radio.upload_volume = False
    radio.upload_listenbw = False
    radio.upload_sqlscn = False
    radio.upload_sqlhys = False
    return True

def min_max_def(value, min_val, max_val, default):
    """returns value if in bounds or default otherwise"""
    if min_val is not None and value < min_val:
        return default
    if max_val is not None and value > max_val:
        return default
    return value


def list_def(value, lst, default):
    """return value if is in the list, default otherwise"""
    if isinstance(default, str):
        default = lst.index(default)
    if value < 0 or value >= len(lst):
        return default
    return value    

@directory.register
class UVK5RadioEgzumer(chirp_common.CloneModeRadio):
    """Quansheng UV-K1 running Falcon firmware."""
    VENDOR = "Quansheng"
    MODEL = "UV-K1 / UV-K5 V3 Falcon v3.0.0"
    BAUD_RATE = 38400
    NEEDS_COMPAT_SERIAL = False
    FIRMWARE_VERSION = ""
    MEM_SIZE =      0x144001    # size of all memory CHANNEL_LOCATION

# this change to send power level chan in the calibration but under macos it give error
# bugfix calibration : put in comment next line: upload_calibration = False
    upload_calibration = False

# advanced settings too
    upload_advanced = False
    upload_volume = False

    upload_listenbw = False
    upload_sqlscn = False
    upload_sqlhys = False

    def _version_string(self):
        # A live release identity takes precedence over the EEPROM defaults/schema tag.
        if self.FIRMWARE_VERSION:
            return self.FIRMWARE_VERSION
        if hasattr(self, "_memobj"):
            saved = str(self._memobj.version.version).split("\x00", 1)[0].strip()
            if saved and saved != "\xff" * 16:
                return saved
        return self.FIRMWARE_VERSION

    def _supports_navigation(self):
        return any(v in self._version_string() for v in ("2.0.1", "2.2.0", "2.2.1", "2.3.0", "2.3.1", "3.0.0", "1.2.1-SE", "1.3.0"))

    def _is_legacy_battery(self):
        return "1.1.6" in self._version_string()

    def _battery_options(self):
        if self._is_legacy_battery():
            return LEGACY_BATTYPE_LIST, LEGACY_BATTYPE_VALUES
        return BATTYPE_LIST, BATTYPE_VALUES

    def _list_index(self, stored):
        stored = int(stored)
        if stored == MONITOR_ID:
            return MR_CHANNELS_LIST  # OFF + lists 1..25 + Monitor
        return stored if 0 <= stored < MR_CHANNELS_LIST else 0

    def _list_storage(self, index):
        return MONITOR_ID if index == MR_CHANNELS_LIST else index

    def _vfo_memory(self, index):
        channel = int(self._memobj.ScreenChannel_B if index else self._memobj.ScreenChannel_A)
        if 0 <= channel < MR_CHANNELS_MAX:
            return self._memobj.channel[channel]
        band = channel - MR_CHANNELS_MAX
        return self._memobj.vfo_channel[(band if 0 <= band < 7 else 0) * 2 + index]

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
    
    def _get_fmlist_name(self, index):
        if not hasattr(self, "_memobj") or not 0 <= index < FM_RADIOS_MAX:
            return ""
        try:
            name_obj = self._memobj.fmradiolist[index].name
        except AttributeError:
            return ""
        chars = []
        for char_element in name_obj:
            val = int(char_element)
            if val in (0x00, 0xFF):
                break
            if 32 <= val <= 126:
                chars.append(val)
        return bytes(chars).decode("ascii", errors="ignore").strip()

    def _get_scanlist_name(self, index):
        if not hasattr(self, "_memobj") or not 0 <= index < MR_CHANNELS_LIST - 1:
            return ""

        try:
            name_obj = self._memobj.listname[index].name
        except AttributeError:
            return ""

        chars = []
        for char_element in name_obj:
            val = int(char_element)
            if val in (0x00, 0xFF):
                break
            if 32 <= val <= 126:
                chars.append(val)

        return bytes(chars).decode("ascii", errors="ignore").strip()

    def _get_scanlist_display_list(self):
        scanlists = ["OFF"]

        for idx in range(MR_CHANNELS_LIST - 1):
            name = self._get_scanlist_name(idx)
            if name:
                scanlists.append(f"{name} [{idx + 1}]")
            else:
                scanlists.append(SCANLIST_LIST[idx + 1])

        scanlists.append(SCANLIST_LIST[-1])   # "Monitor"
        return scanlists

    def _get_scanlist_select_display_list(self):
        scanlists = []

        for idx in range(MR_CHANNELS_LIST - 1):
            name = self._get_scanlist_name(idx)
            if name:
                scanlists.append(f"{name} [{idx + 1}]")
            else:
                scanlists.append(SCANLIST_SELECT_LIST[idx])

        scanlists.append(SCANLIST_SELECT_LIST[-1])   # "Monitor"
        return scanlists

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = \
            'This is a custom driver for Quansheng UV-K1 Falcon. ' \
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
        rf.valid_name_length = 8
        rf.valid_power_levels = UVK5_POWER_LEVELS
        rf.valid_duplexes = ["", "-", "+"]

        steps = STEPS.copy()
        steps.sort()
        rf.valid_tuning_steps = steps

        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone", "Tone->DTCS", "DTCS->Tone",
                                "->Tone", "->DTCS", "DTCS->", "DTCS->DTCS"]

        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_modes = ["FM", "NFM", "AM", "NAM", "USB"]

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
    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    # Do an upload of the radio to the serial port
    def sync_out(self):
        do_upload(self)

    # Convert the raw byte array into a memory object structure
    def process_mmap(self):
        mmap_bytes = self._mmap.get_packed()
        if len(self._mmap.get_packed()) < PROG_SIZE:
            raise errors.RadioError(
                "Image is too small for this driver "
                f"(got 0x{len(self._mmap.get_packed()):X} bytes, "
                f"need 0x{PROG_SIZE:X}). Legacy images from older drivers are "
                "not compatible — re-download from the radio."
            )
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
            msg = f"The frequency {mem.freq/1000000.0:.4f}MHz " \
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
            tmpscn = self._list_index(_mem3.scanlist)
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
            rs = RadioSetting("busyChLockout", "BusyCL", val)
            mem.extra.append(rs)

            val = RadioSettingValueBoolean(False)
            rs = RadioSetting("frev", "FreqRev", val)
            mem.extra.append(rs)

            val = RadioSettingValueList(COMPANDER_LIST)
            rs = RadioSetting("compander", "Compander", val)
            mem.extra.append(rs)

            val = RadioSettingValueList(self._get_scanlist_display_list())
            rs = RadioSetting("scanlists", "Scanlists", val)
            mem.extra.append(rs)

            if number >= BAND_FIRST:
                mem.extra["scanlists"].value.set_mutable(False)
                mem.extra.append(RadioSetting("band_stop", "Band stop (MHz)",
                                              RadioSettingValueString(0, 12, "")))
            return mem

        _mem2 = self._memobj.channelname[ch_num]
        for char in _mem2.name:
            if str(char) == "\xFF" or str(char) == "\x00":
                break
            mem.name += str(char)
        mem.name = mem.name.rstrip()[:8]

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
            mem.mode = temp_modes[temp_modul]
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
        txpower = int(_mem.txpower)
        power_index = POWER_VALUES.index(txpower) if txpower in POWER_VALUES else 0
        mem.power = UVK5_POWER_LEVELS[power_index]

        # We'll consider any blank (i.e. 0MHz frequency) to be empty
        if (_mem.freq == 0xffffffff) or (_mem.freq == 0):
            mem.empty = True
        else:
            mem.empty = False

        mem.extra = RadioSettingGroup("Extra", "extra")

        # BusyCL
        val = RadioSettingValueBoolean(_mem.busyChLockout)
        rs = RadioSetting("busyChLockout", "Busy Ch Lockout (BusyCL)", val)
        rs.set_doc('BusyCL: If the channel is Busy, do not allow TX')
        mem.extra.append(rs)

        # Frequency reverse
        val = RadioSettingValueBoolean(_mem.freq_reverse)
        rs = RadioSetting("frev", "Reverse Frequencies (R)", val)
        rs.set_doc('R: Is this needs to be reversed?')
        mem.extra.append(rs)

        # Compander
        val = RadioSettingValueList(COMPANDER_LIST, None, tmp_comp)
        rs = RadioSetting("compander", "Compander (Compnd)", val)
        rs.set_doc('Compnd: Do you want to compand on this frequency?')
        mem.extra.append(rs)

        val = RadioSettingValueList(self._get_scanlist_display_list(), None, tmpscn)
        rs = RadioSetting("scanlists", "Scanlists (SList)", val)
        rs.set_doc('SList: Is this frequency is part of a scan list?')
        mem.extra.append(rs)

        if mem.number >= BAND_FIRST:
            mem.name = mem.name[:BAND_NAME_LEN]
            mem.duplex = ""
            stop = int(_mem.offset) * 10
            rs = RadioSetting("band_stop", "Band stop (MHz)",
                              RadioSettingValueString(0, 12, f"{stop / 1000000:.5f}"))
            rs.set_doc('Reserved band slot: Frequency is FSTART; this value is FSTOP. Not a TX offset.')
            mem.extra.append(rs)
            mem.extra["scanlists"].value.set_mutable(False)
        return mem

    def set_settings(self, settings):
        _mem = self._memobj
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue

            elname = element.get_name()

            if elname == "auto_roger":
                _mem.auto_roger = int(element.value)
            elif elname == "scrambler":
                _mem.scrambler = int(element.value)
            elif elname == "button_invert":
                if self._supports_navigation() and element.changed():
                    _mem.button_invert = int(element.value)
            elif elname == "listen_bw_fm" and element.changed():
                _mem.listen_bw_fm = int(element.value)
                self.upload_listenbw = True
            elif elname == "listen_bw_am" and element.changed():
                _mem.listen_bw_am = int(element.value)
                _mem.listen_bw_am_tag = 0xB1
                self.upload_listenbw = True

            elif elname == "vfo_scan_sql" and element.changed():
                _mem.scan_sql_tag = 0xD2
                _mem.vfo_scan_sql = int(element.value)
                self.upload_sqlscn = True
            elif elname == "scan_sql_hys" and element.changed():
                _mem.scan_sql_hys = 0xA0 | int(element.value)
                self.upload_sqlhys = True

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

            # squelch
            elif elname == "squelch":
                _mem.squelch = int(element.value)

            # TOT
            elif elname == "tot":
                _mem.max_talk_time = int(element.value)

            # NOAA autoscan
            elif elname == "noaa_autoscan":
                _mem.scrambler = int(element.value)

            # mic gain
            elif elname == "mic_gain":
                _mem.mic_gain = int(element.value)

            # VOX
            elif elname == "vox":
                _mem.vox_switch = int(element.value) != 0
                if _mem.vox_switch:
                    _mem.vox_level = int(element.value) - 1

            # Channel display mode
            elif elname == "channel_display_mode":
                _mem.channel_display_mode = int(element.value)

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
            # mic_bar
            elif elname == "mic_bar":
                _mem.mic_bar = int(element.value)

            # Batterie txt
            elif elname == "battery_text":
                _mem.battery_text = int(element.value)

            elif elname == "led_mode":
                _mem.led_mode = LED_VALUES[int(element.value)]
            elif elname == "rx_mode" and element.changed():
                _mem.crossband = (sonic_rx_config(_mem.crossband) & 2) | RX_MODE_VALUES[int(element.value)]
            elif elname == "tone_1750":
                _mem.crossband = (sonic_rx_config(_mem.crossband) & 13) | (TONE_1750_VALUES[int(element.value)] << 1)

            elif elname == "meter_style":
                _mem.meter_marker = 0x4D
                _mem.meter_classic = int(element.value)
            elif elname == "audio_am":
                _mem.audio_am = int(element.value)
            elif elname == "set_volume" and element.changed():
                _mem.cal.volumeGain = int(element.value)
                self.upload_volume = True
            elif elname in ("prich1", "prich2", "prich3", "prich4"):
                number=int(elname[-1]);setattr(_mem.sl if number<3 else _mem.ow, f"slPriorCh{number}", int(element.value) | (0xA000 if number>=3 else 0))
            elif elname in ("power_vfo_a", "power_vfo_b"):
                if element.changed():
                    self._vfo_memory(elname == "power_vfo_b").txpower = POWER_VALUES[int(element.value)]

            # Tail tone elimination
            elif elname == "ste":
                _mem.ste = int(element.value)

            # Current State
            elif elname == "current_state":
                _mem.current_state = int(element.value)

            # Beep control
            elif elname == "button_beep":
                _mem.button_beep = int(element.value)

            elif elname == "key_lock":
                _mem.key_lock = int(element.value)

            # Set nav
            elif elname == "set_nav":
                if self._supports_navigation() and element.changed():
                    _mem.set_nav = int(element.value)

            # Auto keypad lock
            elif elname == "auto_keypad_lock":
                _mem.auto_keypad_lock = int(element.value)

            # Power on display mode
            elif elname == "welcome_mode":
                _mem.power_on_dispmode = int(element.value)

            # Power on screen
            elif elname == "power_on_screen":
                _mem.power_on_screen = int(element.value)

            # Keypad Tone
            elif elname == "voice":
                _mem.voice = int(element.value)

            elif elname.startswith("dbm_corr_"):
                if self.upload_advanced:
                    i = int(elname.split("_")[-1])
                    _mem.dbm_corr[i] = int(element.value)

            # Alarm mode
            elif elname == "alarm_mode":
                _mem.alarm_mode = int(element.value)

            # Reminding of end of talk
            elif elname == "roger_beep":
                _mem.roger_beep = ROGER_VALUES[int(element.value)]

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
                if self.upload_advanced:
                    _mem.Battery_type = self._battery_options()[1][int(element.value)]

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

            # FM radios list (freq + name, inspired by scan lists)
            elif elname.startswith("FMFreq_"):
                idx = int(elname.split("_")[1])
                if 0 <= idx < FM_RADIOS_MAX:
                    val = str(element.value).strip()
                    try:
                        val2 = int(float(val) * 10)
                    except Exception:
                        val2 = 0
                    if val2 < FMMIN * 10 or val2 > FMMAX * 10:
                        val2 = 0
                    _mem.fmradiolist[idx].freq = val2

            elif elname.startswith("FMName_"):
                idx = int(elname.split("_")[1])
                if 0 <= idx < FM_RADIOS_MAX:
                    val_str = str(element.value)
                    if val_str:
                        val_bytes = val_str.encode('ascii', 'ignore')[:15]
                        val_bytes = val_bytes + b'\x20' * (15 - len(val_bytes))
                    else:
                        val_bytes = b'\x20' * 15
                    _mem.fmradiolist[idx].name = val_bytes

            # fm radio
            for i in range(1, FM_CHANNELS_MAX + 1):
                freqname = "FM_" + str(i)
                if elname == freqname:
                    val = str(element.value).strip()
                    try:
                        value = Decimal(val) * 10
                        if not value.is_finite() or value != value.to_integral_value():
                            raise InvalidOperation
                        val2 = int(value)
                    except (InvalidOperation, ValueError):
                        if val:
                            raise InvalidValueError("Enter an FM frequency with at most one decimal place")
                        val2 = 0

                    if val2 < FM_MEMORY_MIN*10 or val2 > FM_MEMORY_MAX*10:
                        if val and val2 != 0:
                            raise InvalidValueError("FM memory must be between 87.5 and 108.0 MHz")
                        val2 = 0  # Same empty encoding used by the v1.0 FM editor.
                    _mem.fmfreq[i-1] = val2

            # scanlist stuff
            if elname == "slDef":
                index = int(element.value) + 1
                _mem.sl.slDef = self._list_storage(index)
            elif elname.startswith("listname"):
                idx = int(elname.replace("listname", ""))
                if 0 <= idx < (MR_CHANNELS_LIST - 1):
                    val_str = str(element.value)
                    
                    if val_str:
                        val_bytes = val_str.encode('ascii', 'ignore')[:LIST_NAME_LEN]
                        val_bytes = val_bytes + b'\x20' * (LIST_NAME_LEN - len(val_bytes))
                    else: 
                        val_bytes = b'\xFF' * LIST_NAME_LEN
                    _mem.listname[idx].name = val_bytes

            # Shortcuts

            if elname == "key1_shortpress_action":
                if element.changed():
                    _mem.key1_shortpress_action = KEYACTIONS_LIST.index(str(element.value))

            elif elname == "key1_longpress_action":
                if element.changed():
                    _mem.key1_longpress_action = KEYACTIONS_LIST.index(str(element.value))

            elif elname == "key2_shortpress_action":
                if element.changed():
                    _mem.key2_shortpress_action = KEYACTIONS_LIST.index(str(element.value))

            elif elname == "key2_longpress_action":
                if element.changed():
                    _mem.key2_longpress_action = KEYACTIONS_LIST.index(str(element.value))

            elif elname == "keyM_longpress_action":
                if element.changed():
                    _mem.keyM_longpress_action = KEYACTIONS_LIST.index(str(element.value))

# this change to send power level chan in the calibration but under macos it give error
# bugfix calibration : remove the comment on next 2 line:
#            elif elname == "upload_calibration":
#                self._upload_calibration = bool(element.value)

            elif element.changed() and elname.startswith("_mem.cal."):
                exec(elname + " = element.value.get_value()")

    def get_settings(self):
        _mem = self._memobj

        ValFirm = "Firmware : Falcon v3.0.0"

  
        radio_firmware = RadioSettingGroup("radio_firmwarebasic", ValFirm)

        basic = RadioSettingGroup("basic", "Basic Settings")
        advanced = RadioSettingGroup("advanced", "Advanced Settings")
        keya = RadioSettingGroup("keya", "Programmable Keys")
        scanl = RadioSettingGroup("scn", "Scan Lists")
        unlock = RadioSettingGroup("unlock", "Unlock Settings")
        fmradio = RadioSettingGroup("fmradio", "FM Broadcast Receiver")
        calibration = RadioSettingGroup("calibration", "Calibration")
        
        top = RadioSettings()
        top.append(radio_firmware)
        top.append(basic)
        top.append(advanced)
        top.append(keya)
        top.append(scanl)
        top.append(unlock)
        if _mem.BUILD_OPTIONS.ENABLE_FMRADIO:
            top.append(fmradio)
        top.append(calibration)        
        # helper function
        def append_label(radio_setting, label, descr=""):
            if not hasattr(append_label, 'idx'):
                append_label.idx = 0

            val = RadioSettingValueString(len(descr), len(descr), descr)
            val.set_mutable(False)
            rs = RadioSetting("label" + str(append_label.idx), label, val)
            append_label.idx += 1
            radio_setting.append(rs)

        # Programmable keys
        def get_action(action_num, side_key=True):
            """"get actual key action"""
            has_flashlight = self._memobj.BUILD_OPTIONS.ENABLE_FLASHLIGHT
            has_fm_radio = self._memobj.BUILD_OPTIONS.ENABLE_FMRADIO
            action_num = int(action_num)
            ids = KEYACTIONS_SUPPORTED.copy()
            if not side_key:
                ids.remove(24)  # VFO A/B belongs to F1/F2. 
            if action_num in KEYACTIONS_LEGACY:
                ids.append(action_num)  # Preserve an existing functional assignment.
            if not has_flashlight:
                ids.remove(1)
            if not has_fm_radio:
                ids.remove(6)
            if not self._memobj.BUILD_OPTIONS.ENABLE_VOX:
                ids.remove(26)
            lst = [KEYACTIONS_LIST[i] for i in ids]
            action_num = int(action_num)
            if action_num >= len(KEYACTIONS_LIST) or \
               KEYACTIONS_LIST[action_num] not in lst:
                action_num = 0
            return lst, KEYACTIONS_LIST[action_num]

        val1s = RadioSettingValueList(*get_action(_mem.key1_shortpress_action))
        rs = RadioSetting("key1_shortpress_action",
                          "Side Key 1 Short Press (F1Shrt)", val1s)
        rs.set_doc('F1Shrt: Select the action to do when pressing F1 key for a ' + \
                   'SHORT time, F1 key is located on the left side, first key under PTT')
        keya.append(rs)

        val1l = RadioSettingValueList(*get_action(_mem.key1_longpress_action))
        rs = RadioSetting("key1_longpress_action",
                          "Side Key 1 Long Press (F1Long)", val1l)
        rs.set_doc('F1Long: Select the action to do when pressing F1 key for a ' + \
                   'LONG time, F1 key is located on the left side, first key under PTT')
        keya.append(rs)

        val2s = RadioSettingValueList(*get_action(_mem.key2_shortpress_action))
        rs = RadioSetting("key2_shortpress_action",
                          "Side Key 2 Short Press (F2Shrt)", val2s)
        rs.set_doc('F2Shrt: Select the action to do when pressing F2 key for a ' + \
                   'SHORT time, F2 key is located on the left side, second key under PTT')
        keya.append(rs)

        val2l = RadioSettingValueList(*get_action(_mem.key2_longpress_action))
        rs = RadioSetting("key2_longpress_action",
                          "Side Key 2 Long Press (F2Long)", val2l)
        rs.set_doc('F2Long: Select the action to do when pressing F2 key for a ' + \
                   'LONG time, F2 key is located on the left side, second key under PTT')
        keya.append(rs)

        valm = RadioSettingValueList(*get_action(_mem.keyM_longpress_action, False))
        rs = RadioSetting("keyM_longpress_action",
                          "Menu Key Long Press (M Long)", valm)
        rs.set_doc('M Long: Select the action to do when pressing M key for a ' + \
                   'LONG time, M key is located below the screen on the left')
        keya.append(rs)

        # ----------------- Scan Lists

        scanlist_select_list = self._get_scanlist_select_display_list()
        tmpscanl = max(0, self._list_index(_mem.sl.slDef) - 1)
        val = RadioSettingValueList(scanlist_select_list, None, tmpscanl)
        rs = RadioSetting("slDef", "Default Scan Lists (SList)", val)
        rs.set_doc('SList: Selects which lists are used by the memory scan\n' + \
                    '* Lists 1 to 25, or Monitor.\n')
        scanl.append(rs)

        ch_list = []
        for ch in range(1, MR_CHANNELS_MAX + 1):
            ch_list.append("Channel M" + str(ch))
        ch_list.append("None")

        # List names
        for i in range(MR_CHANNELS_LIST - 1):
            name_obj = _mem.listname[i].name
            
            valid_bytes = []
            for char_element in name_obj:
                val = int(char_element)
                if 32 <= val <= 126:
                    valid_bytes.append(val)
            
            listname = bytes(valid_bytes).decode('ascii', errors='ignore').strip()

            val = RadioSettingValueString(0, LIST_NAME_LEN, listname)
            listname_setting = RadioSetting(
                f"listname{i}", 
                f"Scan List Name {i+1}", 
                val
            )
            listname_setting.set_doc(
                f'Name for scan list {i+1}\n'
                f'Maximum {LIST_NAME_LEN} characters'
            )
            scanl.append(listname_setting)

        # ----------------- Basic settings

        ch_list = []
        for ch in range(1, MR_CHANNELS_MAX + 1):
            ch_list.append("Channel M" + str(ch))
        for bnd in range(1, 8):
            ch_list.append("Band F" + str(bnd))

        # Set_tot SONIC
        tmpsettot = list_def(_mem.set_tot, SET_TOT_EOT_LIST, 0)
        val = RadioSettingValueList(SET_TOT_EOT_LIST, SET_TOT_EOT_LIST[tmpsettot])
        SetTotSetting = RadioSetting("set_tot", "Set TX Timeout Indicator (SetTot)", val)
        SetTotSetting.set_doc('SetTot: Indication of the TX timeout\n' + \
                                '* NONE : no information\n' + \
                                '* SOUND : audio information\n' + \
                                '* VISUAL : screen blinking\n' + \
                                '* ALL : audio and screen blinking')

        # Set_eot SONIC
        tmpseteot = list_def(_mem.set_eot, SET_TOT_EOT_LIST, 0)
        val = RadioSettingValueList(SET_TOT_EOT_LIST, SET_TOT_EOT_LIST[tmpseteot])
        SetEotSetting = RadioSetting("set_eot", "Set End Of Transmission Indicator (SetEot)", val)
        SetEotSetting.set_doc('SetEot: End Of Transmission indication\n' + \
                                '* NONE : no information\n' + \
                                '* SOUND : audio information\n' + \
                                '* VISUAL : screen blinking\n' + \
                                '* ALL : audio and screen blinking')

        # Set_contrast SONIC
        tmpcontrast = min_max_def(_mem.set_contrast, 0, 15, 15)
        val = RadioSettingValueInteger(0, 15, tmpcontrast)
        contrastSetting = RadioSetting("set_contrast", "Set Contrast Level (SetCtr)", val)
        contrastSetting.set_doc('SetCtr: Set the display contrast level from 0 to 15, default is 15')
       
        # Set_inv SONIC
        tmpsetinv = list_def(_mem.set_inv, SET_OFF_ON_LIST, 0)
        val = RadioSettingValueList(SET_OFF_ON_LIST, SET_OFF_ON_LIST[tmpsetinv])
        SetInvSetting = RadioSetting("set_inv", "Invert Display (SetInv)", val)
        SetInvSetting.set_doc('SetInv: black text on light background or light text on black background\n' + \
                              '* OFF : black text (default)\n' + \
                              '* ON : light text')
        # Set_lck, uses
        tmpsetlck = list_def(_mem.set_lck, SET_LCK_LIST, 0)
        val = RadioSettingValueList(SET_LCK_LIST, SET_LCK_LIST[tmpsetlck])
        SetLckSetting = RadioSetting("set_lck", "Lock PTT Key When Keypad Is Locked (SetLck)", val)
        SetLckSetting.set_doc('SetLck: When the keypad is locked, lock also the PTT key')
        
        # Set_tmr SONIC
        tmpsettmr = list_def(_mem.set_tmr, SET_OFF_ON_LIST, 0)
        val = RadioSettingValueList(SET_OFF_ON_LIST, SET_OFF_ON_LIST[tmpsettmr])
        SetTmrSetting = RadioSetting("set_tmr", "Set Timer (SetTmr)", val)
        SetTmrSetting.set_doc('SetTmr: To enable or disable the timer on transmit and receive (on the left of the status bar...)\n' + \
                              '* OFF : Disable \n' + \
                              '* ON : Enable ')

        # Set_off SONIC
        tmpsetoff = 0
        val = RadioSettingValueList(SET_OFF_TMR_LIST, SET_OFF_TMR_LIST[tmpsetoff])
        SetOffSetting = RadioSetting("set_off_tmr", "Set Off (SetOff)", val)
        SetOffSetting.set_doc('SetOff: Put the radio in battery save, when no RX or keypress \n' + \
                              'To inform you that the radio is in this mode, the Led will flashing in RED \n' + \
                              'the Display will be OFF and hardware go in sleepmode')

        # Set_NFM SONIC
        tmpsetnfm = list_def(_mem.set_nfm, SET_NFM_LIST, 0)
        val = RadioSettingValueList(SET_NFM_LIST, SET_NFM_LIST[tmpsetnfm])
        SetNFMSetting = RadioSetting("set_nfm", "Set NFM (SetNFM)", val)
        SetNFMSetting.set_doc('SetNFM: Set Narrow FM bandwidth \n' + \
                              '* 12.5 kHz \n' + \
                              '* 6.25 kHz')

        # Set_RXA SONIC
        tmpsetrxa = list_def(_mem.set_rxa, SET_RXA_LIST, 0)
        val = RadioSettingValueList(SET_RXA_LIST, SET_RXA_LIST[tmpsetrxa])
        SetRxASetting = RadioSetting("set_rxa", "Set RxA (SetRxA)", val)
        SetRxASetting.set_doc('SetRxA: Set Rx FM Audio profile \n' + \
                              '* FLAT \n' + \
                              '* CLEAN \n' + \
                              '* MID \n' + \
                              '* BOOST \n' + \
                              '* MAX')

        # Set_KEY SONIC
        tmpsetkey = list_def(_mem.set_key, SET_KEY_LIST, 0)
        val = RadioSettingValueList(SET_KEY_LIST, SET_KEY_LIST[tmpsetkey])
        SetKEYSetting = RadioSetting("set_key", "Set KEY (SetKEY)", val)
        SetKEYSetting.set_doc('SetKEY: Set KEY to enable RescueOps mode')

        # Set_Menu_Lock SONIC
        tmpsetmenulock = list_def(_mem.set_menu_lock, SET_OFF_ON_LIST, 0)
        val = RadioSettingValueList(SET_OFF_ON_LIST, SET_OFF_ON_LIST[tmpsetmenulock])
        SetMenuLockSetting = RadioSetting("set_menu_lock", "Set RescueOps", val)
        SetMenuLockSetting.set_doc('Enable or disable RescueOps mode')

        tmpval = int(_mem.set_nav) if self._supports_navigation() and self._mmap[0xA004:0xA005] != b"\xff" else 0
        val = RadioSettingValueList(SET_NAV_LIST, None, tmpval)
        SetMenuNavSetting = RadioSetting("set_nav", "Set Navigation", val)
        SetMenuNavSetting.set_doc('Set Nav: set navigation\n' + \
                                    'LEFT/RIGHT (UV-K1)\n ' \
                                    'or UP/DOWN (UV-K5 V3)')

        tmpsq = min_max_def(_mem.squelch, 0, 9, 1)
        val = RadioSettingValueInteger(0, 9, tmpsq)
        squelch_setting = RadioSetting("squelch", "Squelch (Sql)", val)
        squelch_setting.set_doc('Sql: Squelch sensitivity level from 0 to 9, 0 to disable squelch')
        
        ch_list = []
        for ch in range(1, MR_CHANNELS_MAX + 1):
            ch_list.append("Channel M" + str(ch))

        val = RadioSettingValueBoolean(_mem.key_lock)
        keypad_lock_setting = RadioSetting("key_lock", "Keypad Locked", val)
        keypad_lock_setting.set_doc('Keypad locked: Lock the keypad now')
        
        tmpval = list_def(_mem.auto_keypad_lock, AUTO_KEYPAD_LOCK_LIST, 1)
        val = RadioSettingValueList(AUTO_KEYPAD_LOCK_LIST, None, tmpval)
        auto_keypad_lock_setting = RadioSetting("auto_keypad_lock", "Auto Lock Keypad After Delay (KeyLck)", val)
        auto_keypad_lock_setting.set_doc('KeyLck: Keypad lock\n' + \
                                         'OFF (no keypad lock)\n' \
                                         'or Delay (from 15s to 10m) of inactivity')
                                         
        tmptot = list_def(_mem.max_talk_time,  TALK_TIME_LIST, 1)
        val = RadioSettingValueList(TALK_TIME_LIST, None, tmptot)
        tx_t_out_setting = RadioSetting("tot", "Max TX Timeout (TxTOut)", val)
        tx_t_out_setting.set_doc('TxTOut: Select the TX time limit\n' + \
                                 'See option (SetTot) of SONIC')

        tmpbatsave = list_def(_mem.battery_save, BATSAVE_LIST, 5)
        val = RadioSettingValueList(BATSAVE_LIST, None, tmpbatsave)
        bat_save_setting = RadioSetting("battery_save", "Battery Saver (BatSav)", val)
        bat_save_setting.set_doc('BatSav: Battery saver option, ratio between active time and sleep time')
               
        tmpmicgain = list_def(_mem.mic_gain, MIC_GAIN_LIST, 3)
        val = RadioSettingValueList(MIC_GAIN_LIST, None, tmpmicgain)
        mic_gain_setting = RadioSetting("mic_gain", "Mic Gain (Mic)", val)
        mic_gain_setting.set_doc('Mic: Set the microphone sensitivity level (Gain)')

        tmpvox = _mem.vox_level + 1 if _mem.vox_switch and _mem.vox_level < 10 else 0
        val = RadioSettingValueList(VOX_LIST, None, tmpvox)
        vox_setting = RadioSetting("vox", "VOX (VOX)", val)
        vox_setting.set_doc('VOX: Voice-operated transmission, OFF or sensitivity level 1 to 10')
        
        val = RadioSettingValueBoolean(_mem.mic_bar)
        mic_bar_setting = RadioSetting("mic_bar", "Microphone Level Bar Display (MicBar)", val)
        mic_bar_setting.set_doc('MicBar: Display the microphone level bar while transmitting')
        tmpdispmode = list_def(_mem.power_on_dispmode, WELCOME_LIST, 0)
        val = RadioSettingValueList(WELCOME_LIST, None, tmpdispmode)

        tmpbattxt = list_def(_mem.battery_text, BAT_TXT_LIST, 3)
        val = RadioSettingValueList(BAT_TXT_LIST, None, tmpbattxt)
        bat_txt_setting = RadioSetting("battery_text", "Battery Level Display (BatTXT)", val)
        bat_txt_setting.set_doc('BatTXT: Display additional battery info on the status bar\n' + \
                                '* VOLTAGE + PERCENT : Voltage and remaining percentage\n' + \
                                '* PERCENT : Percentage of remaining power\n' + \
                                '* VOLTAGE : Voltage\n' + \
                                '* NONE : Nothing')
        val = RadioSettingValueList(LED_LIST, current_index=LED_VALUES.index(int(_mem.led_mode)))
        led_setting = RadioSetting("led_mode", "Light (LIGHT)", val)
        led_setting.set_doc('Long Menu toggles Light. A new long press within 3 seconds cycles On / SOS / Beacon; after 3 seconds it switches Off.')

        rx_config = sonic_rx_config(_mem.crossband)
        val = RadioSettingValueList(RX_MODE_LIST, current_index=(3 if rx_config & 8 else 2) if rx_config & 4 else rx_config & 1)
        rx_mode_setting = RadioSetting("rx_mode", "RxMode", val)
        rx_mode_setting.set_doc('SINGLE / DUAL / OV WATCH / ML WATCH. Overwatch monitors both VFOs and up to four priority channels.')

        val = RadioSettingValueList(TONE_1750_LIST,
                                   current_index=TONE_1750_VALUES.index((rx_config >> 1) & 1))
        tone_setting = RadioSetting("tone_1750", "Tone", val)
        tone_setting.set_doc('1750: enable the tone with PTT + SK2. OFF: disable it.')

        tmpback = list_def(_mem.backlight_time, BACKLIGHT_LIST, 0)
        val = RadioSettingValueList(BACKLIGHT_LIST, None, tmpback)
        back_lt_setting = RadioSetting("backlight_time", "Backlight Time (BLTime)", val)
        back_lt_setting.set_doc('BLTime: Backlight duration, how long the backlight will stay on\n' + \
                                'after the end of an action')


        tmpback = list_def(_mem.backlight_min, BACKLIGHT_LVL_LIST, 0)
        val = RadioSettingValueList(BACKLIGHT_LVL_LIST, None, tmpback)
        bl_min_setting = RadioSetting("backlight_min", "Minimum Backlight Level (BLMin)", val)
        bl_min_setting.set_doc('BLMin: Minimum backlight brightness, when the screen backlight turns off\n' + \
                               'it will dim to this value')


        tmpback = list_def(_mem.backlight_max, BACKLIGHT_LVL_LIST, 10)
        val = RadioSettingValueList(BACKLIGHT_LVL_LIST, None, tmpback)
        bl_max_setting = RadioSetting("backlight_max", "Maximum Backlight Level (BLMax)", val)
        bl_max_setting.set_doc('BLMax: Maximum backlight brightness, when the screen backlight turns on\n' + \
                               'it will light up to this value')

        
        tmpback = list_def(_mem.backlight_on_TX_RX, BACKLIGHT_TX_RX_LIST, 0)
        val = RadioSettingValueList(BACKLIGHT_TX_RX_LIST, None, tmpback)
        blt_trx_setting = RadioSetting("backlight_on_TX_RX", "Backlight on TX/RX (BLTxRx)", val)
        blt_trx_setting.set_doc('BLTxRx : Backlight activation on TX or RX or both TX and RX or no backlight at all\n' + \
                                '* OFF : OFF in all cases\n' + \
                                '* TX : turn ON when TX only\n' + \
                                '* RX : ON when RX only\n' + \
                                '* TX/RX : ON when TX and RX')

        stored_roger = int(_mem.roger_beep)
        tmpalarmmode = ROGER_VALUES.index(stored_roger) if stored_roger in ROGER_VALUES else 0
        val = RadioSettingValueList(ROGER_LIST, None, tmpalarmmode)
        roger_setting = RadioSetting("roger_beep", "End Of Transmission Beep (Roger)", val)
        roger_setting.set_doc('Roger: sound transmitted at the end of TX. Stored IDs remain unchanged.')


        val = RadioSettingValueBoolean(_mem.ste)
        ste_setting = RadioSetting("ste", "Squelch Tail Elimination (STE)", val)
        ste_setting.set_doc('STE: Squelch tail eliminator, eliminates noise at the end of a transmission')

        tmprte = list_def(_mem.rp_ste, RTE_LIST, 0)
        val = RadioSettingValueList(RTE_LIST, None, tmprte)
        rp_ste_setting = RadioSetting("rp_ste", "Repeater Squelch Tail Elimination (RP STE)", val)
        rp_ste_setting.set_doc('RP STE: Repeater squelch tail eliminator')

        
        tmpalarmmode = list_def(_mem.alarm_mode, ALARMMODE_LIST, 0)
        val = RadioSettingValueList(ALARMMODE_LIST, None, tmpalarmmode)
        alarm_setting = RadioSetting("alarm_mode", "Alarm Mode", val)

        # ----------------- Extra settings

       # Upload Advanced Settings checkbox
        val = RadioSettingValueBoolean(False)

        def validate_upload_advanced(value):
            if value and not self.upload_advanced:
                msg = "This will overwrite advanced settings on the radio."
                ret = wx.MessageBox(msg, "Warning", wx.OK | wx.CANCEL |
                                    wx.CANCEL_DEFAULT | wx.ICON_WARNING)
                value = ret == wx.OK
            self.upload_advanced = value
            return value

        val.set_validate_callback(validate_upload_advanced)
        rs = RadioSetting("upload_advanced", "Upload Advanced Settings", val)
        rs.set_doc('Check this box to upload the advanced settings to the radio.\n'
                   'Covers: Battery Type and S-meter corrections. Navigation does not require this checkbox.\n'
                   'Leave unchecked to preserve existing values on the radio.')
        advanced.append(rs)

        # Display the actual downloaded profile. ID 3 was retired in 2.0.0-OA.
        battery_names, battery_ids = self._battery_options()
        stored_battery = int(_mem.Battery_type)
        if stored_battery == 3 and not self._is_legacy_battery():
            stored_battery = 5
        selected_battery = battery_ids.index(stored_battery) if stored_battery in battery_ids else battery_ids.index(5)
        val = RadioSettingValueList(battery_names, current_index=selected_battery)
        bat_type_setting = RadioSetting("Battery_type", "Battery Type (BatTyp)", val)
        bat_type_setting.set_doc('BatTyp: profiles supported by the downloaded firmware.\n'
                                 'Check Upload Advanced Settings to apply a change.\n'
                                 'The custom Falcon curve is offered only for 1.1.6.')

        advanced.append(bat_type_setting)
        advanced.append(SetMenuNavSetting)

        # S-meter dBm correction per band
        dBmCorrDefault = [-15, -16, -10, -4, -7, -6, -1]
        band_names = ["Band 1: < 108 MHz", "Band 2: 108 - 137 MHz", "Band 3: 137 - 174 MHz",
                      "Band 4: 174 - 350 MHz", "Band 5: 350 - 400 MHz", "Band 6: 400 - 470 MHz", "Band 7: > 470 MHz"]

        for i, (name, default) in enumerate(zip(band_names, dBmCorrDefault)):
            tmp = int(_mem.dbm_corr[i])
            if tmp not in range(-64, 65):
                tmp = default
            val = RadioSettingValueInteger(-64, 64, tmp)
            rs = RadioSetting(f"dbm_corr_{i}", f"S-meter correction {name} (dBm)", val)
            rs.set_doc(f'S-meter dBm correction for {name}.\n'
                       f'Default: {default} dBm.\n'
                       f'Inject -93 dBm and adjust until display reads -93 dBm.')
            advanced.append(rs)

        # ----------------- FM radio

        append_label(fmradio, "UV-K1 FM memories (keys 1-9)", "Frequency (MHz)")

        for i in range(1, FM_CHANNELS_MAX + 1):
            fmfreq = _mem.fmfreq[i-1]/10.0
            freq_name = str(fmfreq)
            if fmfreq < FM_MEMORY_MIN or fmfreq > FM_MEMORY_MAX:
                freq_name = ""
            rs = RadioSetting("FM_" + str(i), "Ch " + str(i),
                              RadioSettingValueString(0, 5, freq_name))
            rs.set_doc(f'UV-K1 FM memory {i}, recalled by key {i} in FM mode.\n' +
                       '87.5 to 108.0 MHz, one decimal place. Leave empty to clear.')
            fmradio.append(rs)
        append_label(fmradio, "=" * 6 + " FM Station Name Directory (50) " + "=" * 300, "=" * 300)

        for i in range(FM_RADIOS_MAX):
            rec_freq = int(_mem.fmradiolist[i].freq) / 10.0
            freq_name = str(rec_freq)
            if rec_freq < FMMIN or rec_freq > FMMAX:
                freq_name = ""
            rs = RadioSetting(f"FMFreq_{i}", f"Radio {i+1} Freq",
                              RadioSettingValueString(0, 5, freq_name))
            rs.set_doc('FM Broadcast frequency in MHz, example: 87.8\n' +
                       'Leave empty (or 0) to clear the slot.')
            fmradio.append(rs)

            station_name = self._get_fmlist_name(i)
            rs = RadioSetting(f"FMName_{i}", f"Radio {i+1} Name",
                              RadioSettingValueString(0, 15, station_name))
            rs.set_doc('Station name displayed on the FM screen when this ' +
                       'frequency is selected. Maximum 15 characters.')
            fmradio.append(rs)
        # ----------------- Unlock settings

        # F-LOCK: stored IDs match the firmware enum.
        tmpflock = list_def(_mem.int_flock, FLOCK_LIST, 0)
        val = RadioSettingValueList(FLOCK_LIST, None, tmpflock)
        f_lock_setting = RadioSetting("int_flock", "TX Frequency Lock (F Lock)", val)
        f_lock_setting.set_doc('F Lock: Sets the TX frequency band plan')
        
        # ----------------- Driver Info

        if self.FIRMWARE_VERSION == "":
            firmware = "To get the firmware version please download " \
                       "the image from the radio first"
        else:
            firmware = self.FIRMWARE_VERSION

 # ----------------- Calibration

        val = RadioSettingValueBoolean(False)

        def validate_upload_calibration(value):
            if value and not self.upload_calibration:
                msg = "This option may break your radio!!!\n" \
                    "You are doing this at your own risk.\n" \
                    "Make sure you have a working calibration backup.\n" \
                    "Don't use it unless you know what you're doing."
                ret = wx.MessageBox(msg, "Warning", wx.OK | wx.CANCEL |
                                    wx.CANCEL_DEFAULT | wx.ICON_WARNING)
                value = ret == wx.OK
            self.upload_calibration = value
            return value

        val.set_validate_callback(validate_upload_calibration)
        radio_setting = RadioSetting("upload_calibration",
                                     "Upload Calibration", val)
        radio_setting.set_doc('To Upload only the setting in the calibration section to the radio, ' + \
                              'you need to check this box, then upload to radio.')
        calibration.append(radio_setting)

        radio_setting_group = RadioSettingGroup("squelch_calibration",
                                                "Squelch")
        calibration.append(radio_setting_group)

        bands = {"sqlBand1_3": "Frequency Band 1-3",
                 "sqlBand4_7": "Frequency Band 4-7"}
        for bnd, bndn in bands.items():
            append_label(radio_setting_group,
                         "=" * 6 + " " + bndn + " " + "=" * 300, "=" * 300)
            for sql in range(0, 10):
                prefix = "_mem.cal." + bnd + "."
                postfix = "[" + str(sql) + "]"
                append_label(radio_setting_group, "Squelch " + str(sql))

                name = prefix + "openRssiThr" + postfix
                tempval = min_max_def(eval(name), 0, 255, 0)
                val = RadioSettingValueInteger(0, 255, tempval)
                radio_setting = RadioSetting(name, "Open RSSI Threshold", val)
                radio_setting_group.append(radio_setting)

                name = prefix + "closeRssiThr" + postfix
                tempval = min_max_def(eval(name), 0, 255, 0)
                val = RadioSettingValueInteger(0, 255, tempval)
                radio_setting = RadioSetting(name, "Close RSSI Threshold", val)
                radio_setting_group.append(radio_setting)

                name = prefix + "openNoiseThr" + postfix
                tempval = min_max_def(eval(name), 0, 127, 0)
                val = RadioSettingValueInteger(0, 127, tempval)
                radio_setting = RadioSetting(name, "Open Noise Threshold", val)
                radio_setting_group.append(radio_setting)

                name = prefix + "closeNoiseThr" + postfix
                tempval = min_max_def(eval(name), 0, 127, 0)
                val = RadioSettingValueInteger(0, 127, tempval)
                radio_setting = RadioSetting(name, "Close Noise Threshold", val)
                radio_setting_group.append(radio_setting)

                name = prefix + "openGlitchThr" + postfix
                tempval = min_max_def(eval(name), 0, 255, 0)
                val = RadioSettingValueInteger(0, 255, tempval)
                radio_setting = RadioSetting(name, "Open Glitch Threshold",
                                             val)
                radio_setting_group.append(radio_setting)

                name = prefix + "closeGlitchThr" + postfix
                tempval = min_max_def(eval(name), 0, 255, 0)
                val = RadioSettingValueInteger(0, 255, tempval)
                radio_setting = RadioSetting(name, "Close Glitch Threshold",
                                             val)
                radio_setting_group.append(radio_setting)

        radio_setting_group = RadioSettingGroup("rssi_level_calibration",
                                                "RSSI Levels")
        calibration.append(radio_setting_group)

        bands = {"rssiLevelsBands1_2": "1-2 ", "rssiLevelsBands3_7": "3-7 "}
        for bnd, bndn in bands.items():
            append_label(radio_setting_group,
                         "=" * 6 +
                         " RSSI levels for QS original small bar graph, bands "
                         + bndn + "=" * 300, "=" * 300)
            for lvl in [1, 2, 4, 6]:
                name = "_mem.cal." + bnd + ".level" + str(lvl)
                tempval = min_max_def(eval(name), 0, 65535, 0)
                val = RadioSettingValueInteger(0, 65535, tempval)
                radio_setting = RadioSetting(name, "Level " + str(lvl), val)
                radio_setting_group.append(radio_setting)

#

        radio_setting_group = RadioSettingGroup("tx_power_calibration",
                                                "TX power")
        calibration.append(radio_setting_group)

        for bnd in range(0, 7):
            append_label(radio_setting_group, "=" * 6 + " TX power band "
                         + str(bnd+1) + " " + "=" * 300, "=" * 300)
            powers = {"low": "Low", "mid": "Medium", "hi": "High"}
            for pwr, pwrn in powers.items():
                append_label(radio_setting_group, pwrn)
                bounds = ["lower", "center", "upper"]
                for bound in bounds:
                    name = f"_mem.cal.txp[{bnd}].{pwr}.{bound}"
                    tempval = min_max_def(eval(name), 0, 255, 0)
                    val = RadioSettingValueInteger(0, 255, tempval)
                    radio_setting = RadioSetting(name, bound.capitalize(), val)
                    radio_setting_group.append(radio_setting)

#

        radio_setting_group = RadioSettingGroup("battery_calibration",
                                                "Battery")
        calibration.append(radio_setting_group)

        for lvl in range(0, 6):
            name = "_mem.cal.batLvl[" + str(lvl) + "]"
            temp_val = min_max_def(eval(name), 0, 4999, 4999)
            val = RadioSettingValueInteger(0, 4999, temp_val)
            radio_setting = \
                RadioSetting(name, "Level " + str(lvl) +
                             (" (voltage calibration)" if lvl == 3 else ""),
                             val)
            radio_setting_group.append(radio_setting)

        radio_setting_group = RadioSettingGroup("vox_calibration", "Vox Calibration")
        calibration.append(radio_setting_group)

        for lvl in range(0, 10):
            append_label(radio_setting_group, "Level " + str(lvl + 1))

            name = "_mem.cal.vox1Thr[" + str(lvl) + "]"
            val = RadioSettingValueInteger(0, 65535, eval(name))
            radio_setting = RadioSetting(name, "On", val)
            radio_setting_group.append(radio_setting)

            name = "_mem.cal.vox0Thr[" + str(lvl) + "]"
            val = RadioSettingValueInteger(0, 65535, eval(name))
            radio_setting = RadioSetting(name, "Off", val)
            radio_setting_group.append(radio_setting)

        radio_setting_group = RadioSettingGroup("mic_calibration",
                                                "Microphone Sensitivity")
        calibration.append(radio_setting_group)

        for lvl in range(0, 5):
            name = "_mem.cal.micLevel[" + str(lvl) + "]"
            tempval = min_max_def(eval(name), 0, 31, 31)
            val = RadioSettingValueInteger(0, 31, tempval)
            radio_setting = RadioSetting(name, "Level " + str(lvl), val)
            radio_setting_group.append(radio_setting)

        radio_setting_group = RadioSettingGroup("other_calibration", "Other")
        calibration.append(radio_setting_group)

        name = "_mem.cal.xtalFreqLow"
        temp_val = min_max_def(eval(name), -1000, 1000, 0)
        val = RadioSettingValueInteger(-1000, 1000, temp_val)
        radio_setting = RadioSetting(name, "Xtal Frequency Low", val)
        radio_setting_group.append(radio_setting)

        name = "_mem.cal.volumeGain"
        temp_val = min_max_def(eval(name), 0, 63, 58)
        val = RadioSettingValueInteger(0, 63, temp_val)
        radio_setting = RadioSetting(name, "Volume Gain", val)
        radio_setting_group.append(radio_setting)

        name = "_mem.cal.dacGain"
        temp_val = min_max_def(eval(name), 0, 15, 8)
        val = RadioSettingValueInteger(0, 15, temp_val)
        radio_setting = RadioSetting(name, "DAC Gain", val)
        radio_setting_group.append(radio_setting)
      
        

        
        # Falcon v1.0 menu additions.
        for index, label in enumerate(("Power VFO A (POWER)", "Power VFO B (POWER)")):
            stored = int(self._vfo_memory(index).txpower)
            selected = POWER_VALUES.index(stored) if stored in POWER_VALUES else 0
            basic.append(RadioSetting("power_vfo_b" if index else "power_vfo_a", label,
                                      RadioSettingValueList(SET_LOW_LIST, current_index=selected)))
        style = int(_mem.meter_classic) if int(_mem.meter_marker) == 0x4D else 0
        basic.append(RadioSetting("meter_style", "S-meter style (SetMet)",
                                  RadioSettingValueList(METER_STYLE_LIST, current_index=style)))
        priority_choices = []
        for index in range(MR_CHANNELS_MAX):
            record = _mem.channel[index]
            name = str(_mem.channelname[index].name).split("\x00", 1)[0].rstrip(" \xff")
            frequency = int(record.freq)
            if 1800000 <= frequency <= 130000000:
                priority_choices.append(f"{index + 1:04d} {name} {frequency / 100000:.5f}")
            else:
                priority_choices.append(f"{index + 1:04d} (EMPTY)")
        priority_choices.append("OFF")
        for index, stored in enumerate((_mem.sl.slPriorCh1, _mem.sl.slPriorCh2, _mem.ow.slPriorCh3, _mem.ow.slPriorCh4)):
            stored=int(stored)
            if index>=2:stored=(stored&0x1FFF) if (stored&0xE000)==0xA000 else MR_CHANNELS_MAX
            selected = stored if 0 <= stored < MR_CHANNELS_MAX else MR_CHANNELS_MAX
            basic.append(RadioSetting(f"prich{index + 1}", f"Priority channel {index + 1} (PRICH{index + 1})",
                                      RadioSettingValueList(priority_choices, current_index=selected)))
        basic.append(RadioSetting("audio_am", "AM RX audio profile (SetRxA AM)",
                                  RadioSettingValueList(SET_RXA_AM_LIST,
                                                       current_index=min_max_def(int(_mem.audio_am), 0, 2, 0))))
        basic.append(RadioSetting("set_volume", "Digital RX volume (SetVol)",
                                  RadioSettingValueInteger(0, 63, min_max_def(int(_mem.cal.volumeGain), 0, 63, 58))))


        # -------- LAYOUT
        append_label(basic,
                     "=" * 6 + " Start of FALCON Settings." + "=" * 300, "=" * 300)
        basic.append(contrastSetting)
        basic.append(SetInvSetting)
        basic.append(SetLckSetting)
        basic.append(SetTmrSetting)
        basic.append(SetNFMSetting)
        basic.append(SetRxASetting)

        append_label(basic,
                     "=" * 6 + " End of FALCON settings " + "=" * 300, "=" * 300)

        append_label(basic,
                     "=" * 6 + " General settings " + "=" * 300, "=" * 300)

        basic.append(led_setting)
        basic.append(rx_mode_setting)
        basic.append(tone_setting)
        basic.append(squelch_setting)
        scan_sql = int(_mem.vfo_scan_sql)
        if int(_mem.scan_sql_tag) != 0xD2 or scan_sql > 77:
            scan_sql = 30
        scan_sql_setting = RadioSetting("vfo_scan_sql", "SQLSCN", RadioSettingValueList(SCAN_SQL_LIST, current_index=scan_sql))
        scan_sql_setting.set_doc("VFO memory scan and ML WATCH opening threshold, -130 to -53 dbm. Spectrum keeps its own opening threshold.")
        basic.append(scan_sql_setting)
        hys = int(_mem.scan_sql_hys)
        hys = (hys & 15) if (hys & 0xF0) == 0xA0 and (hys & 15) <= 8 else 0
        hys_setting = RadioSetting("scan_sql_hys", "SQLHYS", RadioSettingValueList(SCAN_HYS_LIST, current_index=hys))
        hys_setting.set_doc("Shared VFO memory scan / ML WATCH / Spectrum hysteresis, 0 to 8 dbm. RX closes below the opening threshold minus this margin. Default 0 retains the previous behavior.")
        basic.append(hys_setting)
        basic.append(auto_keypad_lock_setting)
        basic.append(tx_t_out_setting)
        basic.append(bat_save_setting)
        
        append_label(basic,
                     "=" * 6 + " Display settings " + "=" * 300, "=" * 300)

        basic.append(bat_txt_setting)
        basic.append(mic_bar_setting)
        basic.append(RadioSetting("welcome_mode", "Power-on display (POnMsg)",
                                  RadioSettingValueList(WELCOME_LIST,
                                                       current_index=min_max_def(int(_mem.power_on_dispmode), 0, 2, 1))))
        tmpponscr = int(_mem.power_on_screen)
        if self._mmap[0xA00E:0xA00F] == b"\xff" or tmpponscr >= len(PONSCR_LIST):
            tmpponscr = 0
        rs = RadioSetting("power_on_screen", "Power-on screen (PONSCR)",
                          RadioSettingValueList(PONSCR_LIST, current_index=tmpponscr))
        rs.set_doc("Screen opened after power-up. A key held at power-up or a low battery keeps the normal VFO screen.")
        basic.append(rs)
        # Shared EEPROM message fields, also available to OA applications.
        for index in (1, 2):
            text = str(getattr(_mem, "logo_line" + str(index))).strip("\x20\x00\xff") + "\x00"
            text = _getstring(text.encode('ascii', errors='ignore'), 0, 12)
            setting = RadioSetting("logo" + str(index),
                                   "Message Line " + str(index) + " (12 characters max)",
                                   RadioSettingValueString(0, 12, text))
            setting.set_doc("Shared message field; stored but unused by Falcon 1.3.0.")
            basic.append(setting)

        
        append_label(basic, "=" * 6 + " Backlight settings "
                     + "=" * 300, "=" * 300)

        basic.append(back_lt_setting)
        basic.append(bl_min_setting)
        basic.append(bl_max_setting)
        basic.append(blt_trx_setting)

        append_label(basic, "=" * 6 + " Audio related settings "
                     + "=" * 300, "=" * 300)

        basic.append(mic_gain_setting)
        basic.append(vox_setting)
        basic.append(RadioSetting("scrambler", "Scrambler (Scramb)",
                                  RadioSettingValueList(["OFF"] + [f"{2600 + 100 * i}Hz" for i in range(10)],
                                                        current_index=min_max_def(int(_mem.scrambler), 0, 10, 0))))
        basic.append(roger_setting)
        basic.append(RadioSetting("auto_roger", "Auto Roger (AUROG)",
                                  RadioSettingValueList(AUTO_ROGER_LIST, current_index=min_max_def(int(_mem.auto_roger), 0, 6, 0))))
        basic.append(RadioSetting("button_invert", "Invert navigation (BtnInv)",
                                  RadioSettingValueBoolean(bool(_mem.button_invert) if self._supports_navigation() and self._mmap[0xA15C:0xA15D] != b"\xff" else False)))
        basic["button_invert"].value.set_mutable(self._supports_navigation())
        basic["button_invert"].set_doc("Independent navigation inversion, saved across restarts in 2.0.1-OA, 2.2.0-OA, 1.2.1-SE and 1.3.0.")
        SetMenuNavSetting.value.set_mutable(self._supports_navigation())
        SetMenuNavSetting.set_doc("Select LEFT/RIGHT for UV-K1 or UP/DOWN for UV-K5 V3. Applied by a normal upload; advanced settings are not required. Supported by 2.0.1-OA, 2.2.0-OA, 1.2.1-SE and 1.3.0.")
        listen_fm = min_max_def(int(_mem.listen_bw_fm), 0, 2, 0)
        listen_am = int(_mem.listen_bw_am) == 1 if int(_mem.listen_bw_am_tag) == 0xB1 else listen_fm != 0
        basic.append(RadioSetting("listen_bw_fm", "ListenBW FM", RadioSettingValueList(LISTEN_BW_FM_LIST, current_index=listen_fm)))
        basic.append(RadioSetting("listen_bw_am", "ListenBW AM", RadioSettingValueList(LISTEN_BW_AM_LIST, current_index=int(listen_am))))
        basic.append(ste_setting)
        basic.append(rp_ste_setting)
        if _mem.BUILD_OPTIONS.ENABLE_ALARM:
            basic.append(alarm_setting)

        append_label(basic, "=" * 6 + " Radio state " + "=" * 300, "=" * 300)

        basic.append(keypad_lock_setting)
        unlock.append(f_lock_setting)

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
            max_name = BAND_NAME_LEN if memory.number >= BAND_FIRST else 8
            tag = memory.name[:max_name].ljust(max_name) + "\x00" * (16 - max_name)
            _mem_chname.name = tag  # Store the alpha tag

        if memory.number >= BAND_FIRST:
            if "band_stop" in memory.extra and str(memory.extra["band_stop"].value).strip():
                try:
                    stop = Decimal(str(memory.extra["band_stop"].value)) * 100000
                    if not stop.is_finite() or stop != stop.to_integral_value():
                        raise InvalidOperation
                    stop = int(stop)
                except (InvalidOperation, ValueError):
                    raise errors.RadioError("Band stop must be a frequency in MHz with at most five decimals")
            else:
                stop = int(_mem_chan.offset)
            if not int(_mem_chan.freq) <= stop <= 130000000:
                raise errors.RadioError("Band stop must be at least FSTART and at most 1300 MHz")
            _mem_chan.offset = stop
            _mem_chan.offsetDir = FLAGS1_OFFSET_NONE

        # tone data
        self._set_tone(memory, _mem_chan)

        # step
        _mem_chan.step = STEPS.index(memory.tuning_step)

        # tx power
        power_index = next((i for i, power in enumerate(UVK5_POWER_LEVELS)
                            if str(memory.power) == str(power)), 0)
        _mem_chan.txpower = POWER_VALUES[power_index]
        # -------- EXTRA SETTINGS

        def get_setting(name, def_val):
            if name in memory.extra:
                return int(memory.extra[name].value)
            return def_val

        _mem_chan.busyChLockout = get_setting("busyChLockout", False)
        _mem_chan.freq_reverse = get_setting("frev", False)
        _mem_attr.compander = get_setting("compander", 0)
        if number < MR_CHANNELS_MAX:
            tmp_val = get_setting("scanlists", 0)
            _mem_attr.scanlist = 0 if memory.number >= BAND_FIRST else self._list_storage(tmp_val)

        return memory
