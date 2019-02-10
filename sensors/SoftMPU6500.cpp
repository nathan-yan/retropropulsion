#include "MPU9250.h"

class SoftMPU6500: public MPU9250 {
    public:
        // A list of scenarios
        float SCENARIO_fromSimulation_gx[350];
        float SCENARIO_fromSimulation_gy[350];
        float SCENARIO_fromSimulation_gz[350];

        float SCENARIO_fromSimulation_ax[350];
        float SCENARIO_fromSimulation_ay[350];
        float SCENARIO_fromSimulation_az[350];

        enum Scenario {
            fromSimulation,
            backAndForth,
            yawThenPitch
        };

        int ts = 0;
        int bound = 350;
        Scenario scenario = fromSimulation;

        int begin() {
            return 1;
        }

        int setAccelRange(AccelRange range) {
            return 1;
        }

        int setGyroRange(GyroRange range) {
            return 1;
        }

        int setDlpfBandwidth(DlpfBandwidth bandwidth) {
            return 1;
        }

        int calibrateGyro(){
            return 1;
        }

        int readSensor() {
            ts++;

            return 1;
        }

        float getAccelX_mss(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_ax[ts % 350];
            }
        }

        float getAccelY_mss(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_ay[ts % 350];
            }
        }

        float getAccelZ_mss(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_az[ts % 350];
            }
        }

        float getGyroX_rads(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_gx[ts % 350];
            }
        }

        float getGyroY_rads(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_gy[ts % 350];
            }
        }

        float getGyroZ_rads(){
            if (scenario == fromSimulation) {
                return SCENARIO_fromSimulation_gz[ts % 350];
            }
        }
}
