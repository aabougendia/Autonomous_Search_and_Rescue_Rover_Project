#ifndef AMG8833_H
#define AMG8833_H

#include <Adafruit_AMG88xx.h>

class AMG8833 {
public:
    AMG8833();
    bool begin();
    void readPixels(float* buffer);
    void printPixels();

    // Detection APIs
    bool detectHuman(float minTemp = 26.0, int minClusterSize = 4);
    bool detectHumanRelative(float delta = 1.2, int minClusterSize = 3);

    // Stats
    float getMaxTemperature();
    float getAverageTemperature();

private:
    Adafruit_AMG88xx sensor;
    float pixels[64];
    float previousPixels[64];

    // Helpers
    int detectHeatClusters(float threshold, bool relative);
    bool isTemporallyConsistent(bool currentDetection);

    bool lastDetections[5] = {false};
    int detectionIndex = 0;
};

#endif
