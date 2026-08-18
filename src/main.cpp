
#include <ArduinoFFT.h>
#include <Adafruit_CircuitPlayground.h>

// note: SerialPrint(s) added for visibility and clarity of performance

// constants
const uint16_t samples = 128;
const double samplingFreq = 50.0;
const double dangerZoneIntensity = 60.0;
const int sampleInterval = 2000;  // interval for each sample set in milliseconds
const int evaluationPeriod = 10 * 60 * 1000;  // total period for evaluation in milliseconds
double vReal[samples], vImag[samples];
unsigned int index = 0, sampleCount = 0, dangerCount = 0;
unsigned long lastTime = 0, lastSampleSetTime = 0;
unsigned long samplingPeriod = 1000000 / samplingFreq;
bool isDeviceRunning = false;
bool isAlarmEnabled = false;

// function declarations
void handleButtonPress();
bool collectSamples();
void performFFT();
double analyzeFFT();
void updateFeedback(double intensity);

/*
set up baud rate to 115200 and initialize constraints for
the Circuit Playground library...also set the imaginary
part of the vector to 0.
*/
void setup() {
    Serial.begin(115200);
    CircuitPlayground.begin();
    CircuitPlayground.clearPixels(); // clear Neopixels to start fresh
    memset(vImag, 0, sizeof(vImag));
    for (int i = 0; i < samples; i++) vImag[i] = 0;
    lastSampleSetTime = millis();
}

/*
loop() handles counting of samples and "danger" analysis.
all incoming samples are said to be within 3-6 Hz frequency...if more
than 60% of these samples are found to be above the current
dangerZoneIntensity value, the alarm would sound. 

the samples are collected and performFFT is called upon the collections. 
the intensity value assigned to the analyzeFFT function's output
would give the maximum allowed intensity based off of the incoming sample magnitudes. the rest
of the system logic evaluates whether the evaluation duration is over (5 minutes
for an entire tremor), and whether its intensity is greater than the specified
dangerZoneIntensity.
*/
void loop() {
    handleButtonPress();  // handle button interactions to start/stop device and toggle alarm (if required)
    if (isDeviceRunning) {
        if (collectSamples()) {  // collect data samples for the FFT
            performFFT();  // perform FFT on the collected data
            double intensity = analyzeFFT();  // analyze FFT data to calculate maximum intensity
            updateFeedback(intensity);  // update Neopixels based on calculated intensity
            // debug output to monitor intensity values
            Serial.print("Intensity: "); Serial.println(intensity);

            if (millis() - lastSampleSetTime >= sampleInterval) {
                if (intensity >= dangerZoneIntensity) {
                    dangerCount++;  // increment count of dangerous samples
                }
                sampleCount++;  // increment total count of samples

                // debug outputs to check into counts of samples and dangerous occurrences
                Serial.print("Sample Count: "); Serial.println(sampleCount);
                Serial.print("Danger Count: "); Serial.println(dangerCount);

                if (millis() - lastSampleSetTime >= evaluationPeriod) {  // check if evaluation period is over
                    double dangerRatio = (double)dangerCount / sampleCount;
                    Serial.print("Danger Ratio: "); Serial.println(dangerRatio);
                    if (dangerRatio >= 0.6 && isAlarmEnabled) {
                        Serial.println("Alarm sounding: Danger level exceeded");
                        // potential additional code to trigger alarm
                        CircuitPlayground.playTone(1000, 500);  // play a 1000 Hz tone for 500 milliseconds
                    } else {
                        Serial.println("Not enough danger signals to sound the alarm.");
                    }
                    // reset counters following evaluation period
                    dangerCount = 0;
                    sampleCount = 0;
                    lastSampleSetTime = millis();
                }
            }
        }
    }
}

/*
collect samples in all of the x,
y, and z directions and compute the overall magnitude
from data pertaining to these three axes...vReal is reset at the start
of each new sampling set.
*/
bool collectSamples() {
    if (micros() - lastTime >= samplingPeriod) {
        lastTime = micros();
        double x = CircuitPlayground.motionX();
        double y = CircuitPlayground.motionY();
        double z = CircuitPlayground.motionZ();
        if (index == 0) {
            for (int i = 0; i < samples; i++) vReal[i] = 0;
        }
        vReal[index] = sqrt(x * x + y * y + z * z);
        index++;
        if (index >= samples) {
            index = 0;
            return true;
        }
    }
    return false;
}

/*
perform appropriate FFT computations for incoming accelerometer
samples...this function is to be later called upon in loop() section for 
all input values.
*/
void performFFT() {
    memset(vImag, 0, sizeof(vImag));
    ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFreq);
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
}

/*
handle the conversion of samples from a frequency to an intensity based
value that will later be used for the Neopixels display. the 
frequency is then compared to the 3-6 Hz frequency range since
that is what is considered to be a Parkinsons tremor.
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
handle ON/OFF controls for the entire device,
as well as for the alarm that sounds. there exist specific
sounds that play when either button is pressed.
*/
void handleButtonPress() {
    if (CircuitPlayground.leftButton()) {
        delay(200);  // debounce delay
        CircuitPlayground.playTone(1000, 500);
        delay(200);  // debounce delay
        CircuitPlayground.playTone(2000, 500);  // play a 1000 Hz tone for 500 milliseconds
        CircuitPlayground.clearPixels(); // clear Neopixels to start afresh
        isDeviceRunning = !isDeviceRunning;
        Serial.println(isDeviceRunning ? "Device started" : "Device stopped");
    }
    if (CircuitPlayground.rightButton()) {
        delay(200);
        CircuitPlayground.playTone(2000, 500);
        isAlarmEnabled = !isAlarmEnabled;
        Serial.println(isAlarmEnabled ? "Alarm enabled" : "Alarm disabled");
    }
}


/*
updates the visual Neopixels data based on tremoring detected
by the board's accelerometer...the greens light up by default if there is little
to no movement, and yellows and reds light up progressively as movement samples approach a "tremor"

greens - low intensity, "safe"
yellows - medium intensity, "mild" movement -- could be approaching a tremor
reds - high intensity, extreme movement, falls in 3-6 Hz range -- is a tremor
*/
void updateFeedback(double intensity) {
    const int lowThreshold = 25;
    const int highThreshold = 60;

    uint8_t red, green, blue;
    if (intensity < lowThreshold) {
        // green color - low intensity
        CircuitPlayground.clearPixels();
        green = 255;
        red = 0;
        blue = 0;
        CircuitPlayground.setPixelColor(4, 0, green, 0);
        CircuitPlayground.setPixelColor(5, 0, green, 0);
    } else if (intensity >= lowThreshold && intensity < highThreshold) {
        // yellow color - transition from green to red
        CircuitPlayground.clearPixels();
        green = 255;
        red = 255;
        blue = 0;
        CircuitPlayground.setPixelColor(2, red, green, blue);
        CircuitPlayground.setPixelColor(3, red, green, blue);
        CircuitPlayground.setPixelColor(4, 0, green, 0);
        CircuitPlayground.setPixelColor(5, 0, green, 0);
        CircuitPlayground.setPixelColor(6, red, green, blue);
        CircuitPlayground.setPixelColor(7, red, green, blue);
    } else {
        // red color - high intensity
        CircuitPlayground.clearPixels();
        red = 255;
        green = 0;
        blue = 0;
        CircuitPlayground.setPixelColor(0, red, green, blue);
        CircuitPlayground.setPixelColor(1, red, green, blue);
        CircuitPlayground.setPixelColor(2, 255, 255, blue);
        CircuitPlayground.setPixelColor(3, 255, 255, blue);
        CircuitPlayground.setPixelColor(4, 0, 255, 0);
        CircuitPlayground.setPixelColor(5, 0, 255, 0);
        CircuitPlayground.setPixelColor(6, 255, 255, blue);
        CircuitPlayground.setPixelColor(7, 255, 255, blue);
        CircuitPlayground.setPixelColor(8, red, green, blue);
        CircuitPlayground.setPixelColor(9, red, green, blue);
    }
}
