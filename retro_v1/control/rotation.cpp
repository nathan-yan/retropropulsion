/* Just for debugging */
#include <iostream>
using namespace std;

class Euler {
    public: 
        float x;
        float y;
        float z;

        float transformationMatrix[3][3];

        enum rotationOrder {xyz, zyx};  // Two useful rotation orders for guidance

        rotationOrder order;

        Euler(rotationOrder order){
            this->order = order;
        }

        void update(Quaternion4 orientation){
            // Shared computation in each rotation order
            float m11 = orientation.a * orientation.a + orientation.b * orientation.b - orientation.c * orientation.c - orientation.d * orientation.d;
            
            float m33 = orientation.a * orientation.a - orientation.b * orientation.b - orientation.c * orientation.c + orientation.d * orientation.d;

            if (order == xyz){
                float m23 = 2 * (orientation.c * orientation.d - orientation.a * orientation.b);
                float m13 = 2 * (orientation.b * orientation.d + orientation.a * orientation.c) 
                float m12 = 2 * (orientation.b * orientation.c - orientation.a * orientation.d);

                x = atan2(-m23, m33);                 // atan(-m23 / m33)
                y = atan2(m13 / sqrt(1 - m13 * m13))  // atan(m13/sqrt(1 - m13^2))
                z = atan2(-m12 / m11)                 // atan(-m12 / m11)
            }else if (order == zyx){
                float m21 = 2 * (orientation.b * orientation.c + orientation.a * orientation.d);
                float m31 = 2 * (orientation.b * orientation.d - orientation.a * orientation.c);
                float m32 = 2 * (orientation.c * orientation.d + orientation.a * orientation.b);

                x = atan2(m32 / m33);
                y = atan2(-m31 / sqrt(1 - m31 * m31));
                z = atan2(m21 / m11);
            }
        }

        Euler order(rotationOrder o){
            if (o == xyz){
                cout << "xyz!" << endl;
            }else if (o == zyx){
                cout << "zyx!" << endl;
            }
        }

}