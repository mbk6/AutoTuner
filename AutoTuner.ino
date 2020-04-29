//AUTOTUNER BY Michael Korenchan, JT Kirages, Adam Aaronson
//This program takes an analog signal from pin A0, uses the included arduinoFFT library to find the most dominant frequency, and generates a smoothed pitch output and intonation.
//The raw peak frequency, smoothed pitch, note name, and intonation are printed to the serial plotter

#include "arduinoFFT.h"
#include <math.h>
#include <Stepper.h>
#include "HashMap.h"

//EMA Stuff - for filtering
float EMA_a = 0.2;      //initialization of EMA alpha
int EMA_S = 0;  

using namespace std;

//Note recognition
#define A_HERTZ 440
#define SEMITONE_RATIO pow(2, 1/12.)
String noteArray[] = {
    "A","Bb","B","C",
    "Db","D","Eb","E",
    "F","Gb","G","Ab"
};

HashMap<int, String> noteMap[12];
noteMap[0](0, "A");
noteMap[1](1, "Bb");
noteMap[2](2, "B");
noteMap[3](3, "C");
noteMap[4](4, "Db");
noteMap[5](5, "D");
noteMap[6](6, "Eb");
noteMap[7](7, "E");
noteMap[8](8, "F");
noteMap[9](9, "Gb");
noteMap[10](10, "G");
noteMap[11](11, "Ab");


//FFT
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1400
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];

//Motor Control
#define motorPin1  8     // IN1 on the ULN2003 driver 1
#define motorPin2  9     // IN2 on the ULN2003 driver 1
#define motorPin3  10    // IN3 on the ULN2003 driver 1
#define motorPin4  11    // IN4 on the ULN2003 driver 1
const int stepsPerRevolution = 2048;
Stepper motor = Stepper(stepsPerRevolution, 8, 10, 9, 11);
 
unsigned int aReadPin = A0;

//Smoothing Parameters
double pitch = -1;
double target = 0;
double smooth_vel = 0;
double smooth_accel = 0;
double peak_vel = 0;
double last_peak = -1;


//Initialize serial output, motor speed, signal variables
void setup() {
    Serial.begin(9600);
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
    motor.setSpeed(5);
    pinMode(A0, INPUT);
    EMA_S = analogRead(aReadPin);
}
 
void loop() {
    /*
     * FFT CODE BASED ON: https://www.norwegiancreations.com/2017/08/what-is-fft-and-how-can-you-implement-it-on-an-arduino/
     */
    //SAMPLING
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();
        int raw = analogRead(aReadPin);
        EMA_S = (EMA_a*raw) + ((1-EMA_a)*EMA_S); //Low-pass filtering
        vReal[i] = EMA_S;
        vImag[i] = 0;
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    /*
     * END CITATION
     */

    //Readjust peak - for some reason the arduinoFFT library returns a frequency a little too large
    peak = (peak - 0.1335) / 1.018;

    //Initialize previous peak record
    if(last_peak < 0) {
      last_peak = peak;
    }

    //Use change in peak to detect unwanted spikes
    peak_vel = peak - last_peak;
    last_peak = peak;

    //Update pitch if negative
    if(pitch < 0) {
      pitch = peak;
    }
    
    //If peak changes too quickly, don't update the target
    if(abs(peak_vel) < 10) {
      target = peak;
    }
    
    //Approach the target smoothly
    smooth_accel = -0.8*smooth_vel + 0.05*(target - pitch);
    smooth_vel += smooth_accel;
    pitch += smooth_vel;

    double intonationLevel = getIntonation(pitch);

   
    //Print smoothed pitch
    Serial.print(pitch);
    Serial.print(" ");
    //Print scaled intonation
    Serial.print(100*intonationLevel);
    Serial.print(" ");
    //Print perfect pitch line for reference
    Serial.print(0);
    Serial.print(" ");
    //Print note name
    Serial.println(getNoteName(pitch));
 
    //Update motor position
    motor.step(stepsPerRevolution * intonationLevel * 0.01);
}


/**
 * Returns a double between -1 and 1
 * Uses A440 12-TET tuning
 * Proximity to -1 indicates pitch being flat
 * Proximity to 0 indicates pitch being in tune
 * Proximity to 1 indicates pitch being sharp
 */
double getIntonation(double inputHertz) {
    double semitones = log(inputHertz / A_HERTZ) / log(SEMITONE_RATIO);
    double sharpness = semitones - floor(semitones);
    
    if (sharpness > 0.5) {
        return -1 + 2 * (sharpness - 0.5);
    } else {
        return 2 * sharpness;
    }
}

/**
 * Returns a string representing the note name, preferring flats
 */
String getNoteName(double inputHertz) {
    int semitones = round(log(inputHertz / A_HERTZ) / log(SEMITONE_RATIO));
    int noteValue = ((semitones % 12) + 12) % 12;
    //return noteArray[noteValue];
    return noteMap.getValueOf(noteValue);
}
