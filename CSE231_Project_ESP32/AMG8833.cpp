#include "AMG8833.h"
#include <Arduino.h>

AMG8833::AMG8833() {}

bool AMG8833::begin() {
    return sensor.begin();
}

void AMG8833::readPixels(float* buffer) {
    sensor.readPixels(pixels);
    for (int i = 0; i < 64; i++) {
        buffer[i] = pixels[i];
    }
}

void AMG8833::printPixels() {
    for (int i = 0; i < 64; i++) {
        Serial.print(pixels[i], 2);
        Serial.print(", ");
        if ((i + 1) % 8 == 0) Serial.println();
    }
    Serial.println();
}

float AMG8833::getMaxTemperature() {
    float maxTemp = pixels[0];
    for (int i = 1; i < 64; i++) {
        if (pixels[i] > maxTemp) maxTemp = pixels[i];
    }
    return maxTemp;
}

float AMG8833::getAverageTemperature() {
    float sum = 0.0;
    for (int i = 0; i < 64; i++) sum += pixels[i];
    return sum / 64.0;
}

float AMG8833::computeAdaptiveDelta() {
    float mean = getAverageTemperature();
    float sumSq = 0.0;
    for (int i = 0; i < 64; i++) {
        float diff = pixels[i] - mean;
        sumSq += diff * diff;
    }
    float stddev = sqrt(sumSq / 64.0);

    // Adaptive, but clamped to a safe and functional range
    float delta = stddev * 1.25 + 0.75;  // tuned formula
    delta = constrain(delta, 1.2, 1.75); // empirically verified range

    Serial.print("Ambient = "); Serial.print(mean, 2);
    Serial.print("  StdDev = "); Serial.print(stddev, 2);
    Serial.print("  Adaptive Delta = "); Serial.println(delta, 2);

    return delta;
}


// --- Static threshold human detection ---
bool AMG8833::detectHuman(float minTemp, int minClusterSize) {
    sensor.readPixels(pixels);
    bool hotMask[8][8] = {false};

    for (int i = 0; i < 64; i++) {
        if (pixels[i] >= minTemp) {
            hotMask[i / 8][i % 8] = true;
        }
    }

    // Detect clusters
    int clusters = detectHeatClusters(minTemp, false);
    bool detected = clusters > 0 && isTemporallyConsistent(true);

    // Update previous frame
    for (int i = 0; i < 64; i++) previousPixels[i] = pixels[i];

    return detected;
}

// --- Ambient-relative human detection ---
bool AMG8833::detectHumanRelative(float delta, int minClusterSize) {
    sensor.readPixels(pixels);

    delta = computeAdaptiveDelta();
    float ambient = getAverageTemperature();
    int clusters = detectHeatClusters(ambient + delta, true);

    bool detected = clusters > 0 && isTemporallyConsistent(true);

    for (int i = 0; i < 64; i++) previousPixels[i] = pixels[i];

    return detected;
}

// --- Cluster detection helper (DFS) ---
int AMG8833::detectHeatClusters(float threshold, bool relative) {
    bool visited[8][8] = {false};
    int clusters = 0;

    auto isHot = [&](int x, int y) {
        float temp = pixels[y * 8 + x];
        return temp >= threshold;
    };

    auto dfs = [&](int x, int y, int &count, auto&& dfs_ref) -> void {
        if (x < 0 || x >= 8 || y < 0 || y >= 8 || visited[y][x] || !isHot(x, y)) return;
        visited[y][x] = true;
        count++;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx != 0 || dy != 0) dfs_ref(x + dx, y + dy, count, dfs_ref);
            }
        }
    };

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if (!visited[y][x] && isHot(x, y)) {
                int count = 0;
                dfs(x, y, count, dfs);
                if (count >= 4) clusters++;
            }
        }
    }

    return clusters;
}

// --- Temporal consistency ---
bool AMG8833::isTemporallyConsistent(bool currentDetection) {
    lastDetections[detectionIndex] = currentDetection;
    detectionIndex = (detectionIndex + 1) % 5;

    int count = 0;
    for (bool d : lastDetections) if (d) count++;
    return count >= 3;
}
