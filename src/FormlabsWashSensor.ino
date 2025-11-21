#include <Arduino.h>
#include <FastLED.h>
#include <VL53L0X.h>
#include <Wire.h>

#define CM 10

// --- Hardware Configuration ---
#define DATA_PIN 3
#define NUM_LEDS 42

// --- Physical Layout ---
#define INDICATOR_WIDTH 9
#define FIRST_LED_DISTANCE_OFFSET (12 * CM)  // First LED is 12cm in front of sensor
#define SENSOR_DETECTION_OFFSET (2 * CM)     // Sensor hits object 2cm past real edge of wash station
#define LED_SPACING 33                       // Pixel-to-pixel distance (mm)

// --- Range Settings ---
#define WALL_DISTANCE (155 * CM)
#define WALL_MARGIN (2 * CM)
#define LONG_RANGE_MODE

// --- Timing & Filtering ---
#define FILTER_ALPHA 0.05          // Lower = smoother but slower
#define MAX_INVALID_READINGS 15    // ~0.5s before considering signal lost
#define PRESENCE_TIMEOUT 15000     // 15s wait before activating LEDs
#define HYSTERESIS_THRESHOLD 0.90  // How far (LEDs) to move before shifting indicator

// --- Animation Settings ---
#define BREATH_BPM 20
#define BREATH_MIN 40
#define BREATH_MAX 255
#define PULSE_INTERVAL 6000
#define PULSE_SPEED_PX_MS 0.04f

// --- Global Objects ---
VL53L0X sensor;
CRGB leds[NUM_LEDS];

// --- State Structures ---

struct SensorState {
    float smoothedDistance = -1;  // -1 indicates uninitialized
    int invalidCount = 0;
};

struct LogicState {
    unsigned long presenceStartTime = 0;  // 0 means object not present
    int currentCenterIndex = -1;          // -1 means no valid position
};

struct PulseState {
    unsigned long lastTrigger = 0;
    unsigned long startTime = 0;
    bool active = false;
};

SensorState sensorState;
LogicState logicState;
PulseState pulseState;

// --- Function Prototypes ---
void setupSensor();
bool updateSensorData();
void resetSystemState();
bool isObjectPresentForDuration();
int calculateLedIndex(float distanceMm);
void renderVisuals(int centerIndex);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    setupSensor();
}

void loop() {
    // 1. Read and filter sensor data
    // Returns false if the sensor has been invalid (error/out of range) for too long
    if (!updateSensorData()) {
        resetSystemState();
        FastLED.show();
        return;
    }

    // 2. Check if we are hitting the back wall (disable indicator)
    if (sensorState.smoothedDistance >= WALL_DISTANCE - WALL_MARGIN) {
        resetSystemState();
        FastLED.show();
        return;
    }

    // 3. Check 15-second presence timer
    if (!isObjectPresentForDuration()) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        return;
    }

    // 4. Calculate center LED position (physics & hysteresis)
    int centerIndex = calculateLedIndex(sensorState.smoothedDistance);

    // 5. Render effects (breathing, falloff, pulse)
    renderVisuals(centerIndex);

    FastLED.show();
}

// --- Helper Functions ---

void setupSensor() {
    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println(F("Failed to detect and initialize sensor!"));
        while (1) {
        }
    }

#if defined LONG_RANGE_MODE
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

    // 33ms budget for ~30Hz updates
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
}

// Returns true if we have valid data to show (even if it's slightly stale),
// Returns false if data has been invalid for > MAX_INVALID_READINGS.
bool updateSensorData() {
    int rawDistance = sensor.readRangeContinuousMillimeters();
    bool isError = sensor.timeoutOccurred() || rawDistance == 0 || rawDistance > 8000;

    if (isError) {
        sensorState.invalidCount++;
        if (sensorState.invalidCount > MAX_INVALID_READINGS) {
            return false;  // Too many errors, system should reset
        }
        // If errors are few, we implicitly return 'true' but DO NOT update smoothedDistance.
        // This freezes the display at the last known good position.
    } else {
        // Valid reading
        sensorState.invalidCount = 0;
        // EMA Filter
        if (sensorState.smoothedDistance < 0) {
            sensorState.smoothedDistance = rawDistance;
        } else {
            sensorState.smoothedDistance = (FILTER_ALPHA * rawDistance) + ((1.0 - FILTER_ALPHA) * sensorState.smoothedDistance);
        }
        Serial.printf("raw: %i mm (smoothed: %i mm)\n", rawDistance, (int)sensorState.smoothedDistance);
    }
    return true;
}

void resetSystemState() {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    sensorState.smoothedDistance = -1;
    sensorState.invalidCount = 0;
    logicState.presenceStartTime = 0;
    logicState.currentCenterIndex = -1;
}

bool isObjectPresentForDuration() {
    // Initialize timer if this is the first frame of detection
    if (logicState.presenceStartTime == 0) {
        logicState.presenceStartTime = millis();
        if (logicState.presenceStartTime == 0) logicState.presenceStartTime = 1;  // Safety
    }

    return (millis() - logicState.presenceStartTime >= PRESENCE_TIMEOUT);
}

int calculateLedIndex(float distanceMm) {
    int trueDistance = (int)distanceMm - SENSOR_DETECTION_OFFSET;
    float rawIndex = (float)(trueDistance - FIRST_LED_DISTANCE_OFFSET) / LED_SPACING;

    // Hysteresis Logic
    if (logicState.currentCenterIndex == -1) {
        // First run, snap immediately
        logicState.currentCenterIndex = (int)round(rawIndex);
    } else {
        float diff = rawIndex - logicState.currentCenterIndex;

        // Sticky threshold: only move if we cross threshold
        if (diff > HYSTERESIS_THRESHOLD) {
            logicState.currentCenterIndex++;
        } else if (diff < -HYSTERESIS_THRESHOLD) {
            logicState.currentCenterIndex--;
        }

        // Fast movement catch-up
        // Noise spikes < 2.5 LEDs wide won't break the stickiness.
        if (abs(diff) > 2.5) {
            logicState.currentCenterIndex = (int)round(rawIndex);
        }
    }
    return logicState.currentCenterIndex;
}

void renderVisuals(int centerIndex) {
    int halfWidth = INDICATOR_WIDTH / 2;
    uint8_t breathBrightness = beatsin8(BREATH_BPM, BREATH_MIN, BREATH_MAX);
    unsigned long now = millis();

    // 1. Base indicator with quadratic falloff
    for (int i = 0; i < NUM_LEDS; i++) {
        int d = abs(i - centerIndex);

        if (d <= halfWidth) {
            // Calculate falloff (center=1.0, edge=0.0)
            float ratio = (float)d / halfWidth;
            float factor = 1.0f - ratio;
            float quadFactor = factor * factor;  // Quadratic curve

            uint8_t baseVal = (uint8_t)(255 * quadFactor);
            uint8_t finalVal = scale8(baseVal, breathBrightness);

            leds[i] = CRGB(0, finalVal, 0);
        } else {
            leds[i] = CRGB::Black;
        }
    }

    // 2. Pulse Logic
    if (!pulseState.active && (now - pulseState.lastTrigger > PULSE_INTERVAL)) {
        pulseState.active = true;
        pulseState.startTime = now;
        pulseState.lastTrigger = now;
    }

    if (pulseState.active) {
        int offset = (int)((now - pulseState.startTime) * PULSE_SPEED_PX_MS);

        int leftPos = (centerIndex - halfWidth) - offset;
        int rightPos = (centerIndex + halfWidth) + offset;
        bool visible = false;

        if (leftPos >= 0 && leftPos < NUM_LEDS) {
            leds[leftPos] += CRGB::White;
            visible = true;
        }
        if (rightPos >= 0 && rightPos < NUM_LEDS) {
            leds[rightPos] += CRGB::White;
            visible = true;
        }

        // Terminate pulse if it travels off screen
        if (offset > NUM_LEDS && !visible) {
            pulseState.active = false;
        }
    }
}
