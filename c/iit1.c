#include "iit1.h"

float controller(bool tempomatOn, float speed, float pedal)
{
    static bool prevOnoff = false;
    static float baseSpeed = 0;
    static float xD = 0;
    float Kp = 1;
    float Ki = 0.1;
    float output = 0;

    if(tempomatOn == true){
        if (prevOnoff == false){
            baseSpeed = speed;
            xD = pedal / Ki;
        }
        float ek = (baseSpeed - speed);
        output = Kp * ek + Ki* xD + Ki * 0.1 * ek;
        if(output <=1 && output >=0){
            xD += 0.1 * ek;
        } else {
            xD += 0;
            if (output <=0){
                output = 0;
            } else {
                output =1;
            }
        }
    } else {
        output = pedal;
    }
    prevOnoff = tempomatOn;
    return output;
}


int main()
{
    if (!initializeRT()){
        return -1;
    }
    getch();
    terminateRT();
    return 0;
}
