read all
verify chip id  
verify power down state 
0x03 reserved
power ctl 2 
    00 00 01 01
    Headphone channel is ON when the SPK/HP_SW pin, 6, is LO
    Headphone channel is OFF when the SPK/HP_SW pin, 6, is HI
    Speaker channel is ON when the SPK/HP_SW pin, 6, is HI
    Speaker channel is OFF when the SPK/HP_SW pin, 6, is LO

Clocking Control:   
/*
24000 Hz   24000 * 256 =  6 144 000 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 10, 32kGROUP = 0, VIDEOCLK = 0,
32000 Hz   32000 * 256 =  8 192 000 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 01, 32kGROUP = 1, VIDEOCLK = 0,
44100 Hz   44100 * 256 = 11 289 600 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 01, 32kGROUP = 0, VIDEOCLK = 0,
48000 Hz   48000 * 256 = 12 288 000 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 01, 32kGROUP = 0, VIDEOCLK = 0,
88200 Hz   88200 * 256 = 22 579 200 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 00, 32kGROUP = 0, VIDEOCLK = 0,
96000 Hz   96000 * 256 = 24 576 000 Hz MCLK, MCLKDIV2 = 0, AUTO = 0, SPEED[1:0] = 00, 32kGROUP = 0, VIDEOCLK = 0,
*/

clocking ctl 0x81 -> AUTO = 1, MCLKDIV2 = 1, 32kGROUP = 1 if fs 32000

interface ctl 1 >>> LEFT JUSTIFIED!!!
    00x000WL    , left justified, 24 v 16 bit
    M/S = 0, INV_SCLK = 0, Reserved = x, DSP = 0, DACDIF[1:0] = 00, AWL[1:0] = 01 v. 11

interface ctl 2 
    x0xx0xxx,   SCLK=MCLK (?), INV_SWCH = 0, SPK/HP_SW pin not inverted

PASSASEL = 0x00 no inputs selected
PASSBSEL = 0x00 no inputs selected

analog zc and sr settings
    xxxxxxxx, analog settings, dont care

passthrough gang control
    xxxxxxxx, dont care

playback ctl 1
    01100000
    HPGAIN[2:0] = 0.6047, 
    independent volume control, noninverted pcm polarity, master playback mute disabled

misc ctl 
    00000011
    passthrough analog disabled, passthrough mute disabled, freeze disabled, deemphasis disabled
    digital soft ramp and zero cross enabled

playback ctl 2
    0x00
    headphone mute disabled, speaker mute disabled (?), speaker volume setting b=a disabled, 
    speaker channel swap disabled, speaker parallel full bridge output disabled, mute 50/50 disabled

PASSAVOL = 0x00 ignored -> analog passthrough disabled (misc ctl reg)
PASSBVOL = 0x00 ignored -> analog passthrough disabled (misc ctl reg)

PCMA = 0x00, no mute 0 dB
PCMB = 0x00, no mute 0 dB

beep freq on time = 0xXX dont care
beep vol off time = 0xXX dont care

beep tone config
    00000000
    beep off, beep mix enabled, treble 5 kHz, bass 50 Hz, tone control disabled

tone control = 0x00
    treble gain 0 dB, bass gain 0 dB

MSTA = 0x00, 0 dB
MSTB = 0x00, 0 dB

HPA = 0x00, 0 dB
HPB = 0x00, 0 dB

SPKA = 0x01, Muted
SPKB = 0x01, Muted

pcm channel swap
    PCMASWP[1:0] = 00   LineOUTA = Left
    PCMBSWP[1:0] = 00   LineOUTB = Right
    Reserved[3:0] = xx
    00 00 XX XX

limiter ctl 1
    LMAX[2:0] = 000, 0 dB Threshold
    CUSH[2:0] = 000, 0 dB Threshold
    LIMSRDIS = 0 do not override soft ramp settings
    LIMZCDIS = 0 do not override zero cross settings

limiter ctl 2 = 0x7F
    Limiter disabled, attenuate both channels, slowest release rate

limiter attack rate = 0x00 fastest attack

status -> read-only

battery compensation = 00 XX 10 11
    battery compensation disabled, vp monitor disabled, vp reference 3.0 V (schematic)

vp battery level -> read only

speaker status ->read only

charge pump frequency = 0x50
    CHGFREQ[3:0] = N >>> N = 5
    Frequency = (64*Fs) / (N+2)
    ...
    (N+2) * Frequency = 64*Fs
    N+2 = 64*(Fs/Frequency)
    N = 64*(Fs/Frequency) - 2