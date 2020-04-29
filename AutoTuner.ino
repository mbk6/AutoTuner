//This takes an analog signal from pin A0, finds the most dominant frequency, and generates a smoothed pitch output.
//The raw peak frequency, smoothed pitch, note name, and intonation are printed to the serial plotter
//Michael Korenchan, JT Kirages, Adam Aaronson

#include "arduinoFFT.h"
#include <math.h>
#include <Stepper.h>
#include <ArduinoQueue.h>

//QUEUE stuff
ArduinoQueue<double> q;
int q_size = 20;
double average = 0;

//EMA Stuff
float EMA_a = 0.2;      //initialization of EMA alpha
int EMA_S = 0;  

using namespace std;

#define A_HERTZ 440
#define SEMITONE_RATIO pow(2, 1/12.)
 
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1400 

#define motorPin1  8     // IN1 on the ULN2003 driver 1
#define motorPin2  9     // IN2 on the ULN2003 driver 1
#define motorPin3  10    // IN3 on the ULN2003 driver 1
#define motorPin4  11    // IN4 on the ULN2003 driver 1
 
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
unsigned int aReadPin = A0;
unsigned int val;
double pitch = -1;
double target = 0;
double smooth_vel = 0;
double smooth_accel = 0;
double peak_vel = 0;
double last_peak = -1;
double minPeak = 100;
double maxPeak = 650;
const int stepsPerRevolution = 2048;
Stepper motor = Stepper(stepsPerRevolution, 8, 10, 9, 11);

 
double vReal[SAMPLES];
double vImag[SAMPLES];

String noteArray[] = {
    "A","Bb","B","C",
    "Db","D","Eb","E",
    "F","Gb","G","Ab"};

 
void setup() {
    Serial.begin(9600);
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
    motor.setSpeed(5);

    pinMode(A0, INPUT);
    EMA_S = analogRead(aReadPin);
}

bool inRange(double peak) {
  return (peak > minPeak) && (peak < maxPeak);
}
 
void loop() {
   
    //SAMPLING
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();

        int raw = analogRead(aReadPin);
        EMA_S = (EMA_a*raw) + ((1-EMA_a)*EMA_S);
        vReal[i] = EMA_S;
        vImag[i] = 0;
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
    //FFT code based on https://www.norwegiancreations.com/2017/08/what-is-fft-and-how-can-you-implement-it-on-an-arduino/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    
    
    peak = (peak - 0.1335) / 1.018;


//    if(q.item_count() >= q_size) {
//      if(abs(average - peak) < 5) {
//        target = peak;
//      }
//      average -= q.dequeue() / q_size;
//      average += peak / q_size;
//      q.enqueue(peak);
//    }
//    else {
//      q.enqueue(peak);
//      average += peak / q_size;
//    }




    
    if(last_peak < 0) {
      last_peak = peak;
    }
    peak_vel = peak - last_peak;
    last_peak = peak;
    if(pitch < 0) {
      pitch = peak;
    }
    if(abs(peak_vel) < 10) {
      target = peak;
    }
    smooth_accel = -0.8*smooth_vel + 0.05*(target - pitch);
    smooth_vel += smooth_accel;
    pitch += smooth_vel;

    double intonationLevel = getIntonation(pitch);

   
    //Serial.print(peak);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(100*intonationLevel);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(getNoteName(pitch));
 

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
    return noteArray[noteValue];
}
