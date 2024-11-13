#ifndef SINGLE_POLE_IIR_FILTER_H
#define SINGLE_POLE_IIR_FILTER_H

#include <stdint.h>

class SinglePoleIIRFilter {
private:
    uint32_t filteredValue;  // Holds the last filtered value
    float alpha;             // Smoothing factor (0 < alpha <= 1)
    bool isFirstValue;       // Flag to check if the first value is being used

public:
    // Constructor with initialization
    SinglePoleIIRFilter(float smoothingFactor, uint32_t initialValue = 0)
        : filteredValue(initialValue), alpha(smoothingFactor), isFirstValue(false) {}

    // Filter function
    uint32_t filter(uint32_t newValue) {
        if (isFirstValue) {
            filteredValue = newValue;  // Use the first value as-is on the first call
            isFirstValue = false;
        } else {
            filteredValue = static_cast<uint32_t>(alpha * newValue + (1.0f - alpha) * filteredValue);
        }
        return filteredValue;
    }

    // Getter for the current filtered value
    uint32_t getFilteredValue() const {
        return filteredValue;
    }
};

#endif // SINGLE_POLE_IIR_FILTER_H
