# Wearable Parkinsonian Tremor Detector

Parkinson's affects [more than 10 million people worldwide](https://www.parkinson.org/understanding-parkinsons/statistics),
and rest tremor is [the presenting symptom in about 70% of cases](https://www.aafp.org/pubs/afp/issues/2018/0201/p180.html):
shaking in a supported limb that fades once the limb starts moving. Classic
rest tremor sits in a narrow band, 4 to 6 Hz, and that narrowness is the
opening. Ordinary movement spreads its energy across the spectrum while a
tremor concentrates it in one place, so an accelerometer and an FFT are
enough to tell the two apart.

This is that idea on an Adafruit Circuit Playground Classic and nothing else.
No extra sensors, no companion phone, no SD card. Onboard accelerometer,
NeoPixel ring, speaker and two buttons, inside 2.5 KB of RAM. The constraint
was the point: how much of a tremor monitor fits on the hardware already
strapped to the wrist.

## How it works

All three accelerometer axes are read at 50 Hz and reduced to their
magnitude, so how the board sits on the wrist does not matter. Every 128
samples, about 2.6 seconds, a Hamming-windowed FFT runs and the strongest
bin between 3 and 6 Hz is taken as the reading. The band reaches a hertz
below the clinical 4 Hz on purpose, since the bins are 0.39 Hz apart and a
tremor sitting near the boundary should not fall out of the window. Bin 0 is
skipped, since at rest that one is mostly gravity.

The ring shows the reading live. Green at the centre while things are still,
yellow spreading outward as the band gets louder, red across the full ring
once the reading passes the tremor threshold.

The alarm is a slower judgement. A single jolt is not a tremor, so the device
counts how many readings over a 10-minute window sat above the threshold and
sounds the speaker only if at least 60% did. The left button starts and stops
sampling. The right button arms the alarm.

## Build and flash

Needs [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html)
(`pip install platformio`) or the PlatformIO VS Code extension.

```bash
git clone https://github.com/rayedkhan/Wearable-Parkinsonian-Tremor-Detector.git
cd Wearable-Parkinsonian-Tremor-Detector

pio run                       # build
pio run -t upload             # flash a connected Circuit Playground Classic
pio device monitor -b 115200  # watch the readings
```

The two libraries it depends on, Adafruit Circuit Playground and arduinoFFT,
are pulled in on the first build. The firmware lands at 57% of flash and 78%
of RAM on the ATmega32u4.

## Where it stops

There is no clinical validation here. Nothing was tested against patients or
against a reference tremor recording, so the two thresholds, 60 for the
danger band and 25 for the yellow one, are tuned against my own hand and
nothing more. Treating them as diagnostic would be a mistake.

What the build does show is that the whole path holds on an 8-bit AVR: 50 Hz
sampling, a 128-point FFT, band extraction and a live display, running
continuously in half the flash and under 2 KB of RAM, with the frequency
reading tracking shaking as it happens.
