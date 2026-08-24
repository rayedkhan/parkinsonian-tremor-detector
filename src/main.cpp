/*
Rest-tremor detector for the Adafruit Circuit Playground Classic.

Samples the onboard accelerometer at 50 Hz, runs a 128-point FFT over the
magnitude of the three axes, and reads the strongest component in the 3-6 Hz
band, where Parkinsonian rest tremor sits. The NeoPixel ring shows that
reading live; if most of a 10-minute window lands in the danger band and the
alarm is armed, the speaker sounds.

Serial output at 115200 baud mirrors every reading, since the ten pixels
alone cannot show the numbers behind a decision.
*/

#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

const uint16_t samples = 128;                 // power of two, as the FFT requires
const double samplingFreq = 50.0;             // Hz, far above the 6 Hz top of the band of interest
const double dangerZoneIntensity = 60.0;      // 3-6 Hz magnitude at or above which a window counts as tremor
const double mildIntensity = 25.0;            // display-only: where the ring starts turning yellow
const double dangerRatio = 0.6;               // share of danger windows needed to sound the alarm
const int sampleInterval = 2000;              // ms, minimum spacing between counted windows
const int evaluationPeriod = 10 * 60 * 1000;  // ms, length of one alarm decision window

double vReal[samples], vImag[samples];

unsigned int sampleIndex = 0, sampleCount = 0, dangerCount = 0;
unsigned long lastTime = 0, lastSampleSetTime = 0;
unsigned long samplingPeriod = 1000000 / samplingFreq;  // microseconds between accelerometer reads
bool isDeviceRunning = false;
bool isAlarmEnabled = false;

void handleButtonPress();
bool collectSamples();
void performFFT();
double analyzeFFT();
void updateFeedback(double intensity);

void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();
    CircuitPlayground.clearPixels();
    memset(vImag, 0, sizeof(vImag));
    lastSampleSetTime = millis();
}

/*
One pass per accelerometer read. A full window of 128 samples at 50 Hz takes
about 2.6 seconds, so each completed window contributes one reading to the
running tally. After 10 minutes of readings, the alarm fires if at least 60%
of them sat in the danger band.
*/
void loop() {
    handleButtonPress();
    if (isDeviceRunning) {
        if (collectSamples()) {
            performFFT();
            double intensity = analyzeFFT();
            updateFeedback(intensity);
            Serial.print("Intensity: "); Serial.println(intensity);

            if (millis() - lastSampleSetTime >= sampleInterval) {
                if (intensity >= dangerZoneIntensity) {
                    dangerCount++;
                }
                sampleCount++;

                Serial.print("Sample Count: "); Serial.println(sampleCount);
                Serial.print("Danger Count: "); Serial.println(dangerCount);

                if (millis() - lastSampleSetTime >= evaluationPeriod) {
                    double observedRatio = (double)dangerCount / sampleCount;
                    Serial.print("Danger Ratio: "); Serial.println(observedRatio);
                    if (observedRatio >= dangerRatio && isAlarmEnabled) {
                        Serial.println("Alarm sounding: Danger level exceeded");
                        CircuitPlayground.playTone(1000, 500);
                    } else {
                        Serial.println("Not enough danger signals to sound the alarm.");
                    }
                    dangerCount = 0;
                    sampleCount = 0;
                    lastSampleSetTime = millis();
                }
            }
        }
    }
}

/*
Reads one sample per sampling period and stores the magnitude of the three
axes, so the transform sees tremor in any orientation rather than one axis.
Returns true when a full window is ready.
*/
bool collectSamples() {
    if (micros() - lastTime >= samplingPeriod) {
        lastTime = micros();
        double x = CircuitPlayground.motionX();
        double y = CircuitPlayground.motionY();
        double z = CircuitPlayground.motionZ();
        if (sampleIndex == 0) {
            for (int i = 0; i < samples; i++) vReal[i] = 0;
        }
        vReal[sampleIndex] = sqrt(x * x + y * y + z * z);
        sampleIndex++;
        if (sampleIndex >= samples) {
            sampleIndex = 0;
            return true;
        }
    }
    return false;
}

/*
Hamming window before the transform, since a window of accelerometer data is
an arbitrary slice of a continuous signal and the discontinuity at its edges
would otherwise smear energy across the spectrum.

The FFT object is built per call rather than hoisted to a global: the
compiler folds the stack object away, while a global costs 17 bytes of RAM
and 750 bytes of flash on a board already using 78% of its 2.5 KB.
*/
void performFFT() {
    memset(vImag, 0, sizeof(vImag));
    ArduinoFFT<double> FFT(vReal, vImag, samples, samplingFreq);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
}

/*
Strongest magnitude in the 3-6 Hz band, the frequency range of Parkinsonian
rest tremor. Bin 0 is skipped: it holds the DC term, which here is gravity.
*/
double analyzeFFT() {
    double maxIntensity = 0;
    for (int i = 1; i < samples / 2; i++) {
        double frequency = i * samplingFreq / samples;
        if (frequency >= 3 && frequency <= 6) {
            maxIntensity = max(maxIntensity, vReal[i]);
        }
    }
    return maxIntensity;
}

/*
Left button starts and stops sampling, right button arms and disarms the
alarm. Each press answers with a tone, because the wearer cannot see the
serial log.
*/
void handleButtonPress() {
    if (CircuitPlayground.leftButton()) {
        delay(200);  // debounce
        CircuitPlayground.playTone(1000, 500);
        delay(200);
        CircuitPlayground.playTone(2000, 500);
        CircuitPlayground.clearPixels();
        isDeviceRunning = !isDeviceRunning;
        Serial.println(isDeviceRunning ? "Device started" : "Device stopped");
    }
    if (CircuitPlayground.rightButton()) {
        delay(200);  // debounce
        CircuitPlayground.playTone(2000, 500);
        isAlarmEnabled = !isAlarmEnabled;
        Serial.println(isAlarmEnabled ? "Alarm enabled" : "Alarm disabled");
    }
}

/*
The ring fills outward from the two centre pixels as the 3-6 Hz band gets
louder: green while still, yellow through the middle, red once the reading
is at or past the level that counts as tremor.
*/
void updateFeedback(double intensity) {
    CircuitPlayground.clearPixels();

    CircuitPlayground.setPixelColor(4, 0, 255, 0);
    CircuitPlayground.setPixelColor(5, 0, 255, 0);

    if (intensity >= mildIntensity) {
        CircuitPlayground.setPixelColor(2, 255, 255, 0);
        CircuitPlayground.setPixelColor(3, 255, 255, 0);
        CircuitPlayground.setPixelColor(6, 255, 255, 0);
        CircuitPlayground.setPixelColor(7, 255, 255, 0);
    }

    if (intensity >= dangerZoneIntensity) {
        CircuitPlayground.setPixelColor(0, 255, 0, 0);
        CircuitPlayground.setPixelColor(1, 255, 0, 0);
        CircuitPlayground.setPixelColor(8, 255, 0, 0);
        CircuitPlayground.setPixelColor(9, 255, 0, 0);
    }
}
