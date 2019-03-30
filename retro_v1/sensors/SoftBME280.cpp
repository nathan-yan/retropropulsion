#include "BME280.h"

class SoftBME280: public BME280 {
    public:
        bool beginI2C(TwoWire &wirePort = Wire) {
            return true;        // Initialization will always work
        }  
}