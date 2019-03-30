#include "SPIFlash.h"

class SoftSPI: public SPIFlash {
    public:
        int initialize() {
            return 1;
        }
}