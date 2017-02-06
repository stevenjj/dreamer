#include <stdint.h>
#include "PLL.h"

void lightsInit(void);
void lightsUpdate(uint32_t);

int main(void) {
    PLL_Init();
    lightsInit();
    uint32_t magenta = 0x00001717;
    lightsUpdate(magenta);

    uint32_t hue = 0;
    uint32_t hue2 = 0;
    uint32_t color = 0;
    
    while(1) {
        uint32_t spin = 0;
        for(uint32_t i = 0; i < 75000; i++)
            spin = 1;

        hue = (hue + 1)%255;
        hue2 = 255 - hue;
        if(hue2 < 85)
            color = ((255 - hue2*3)/4<<16) + ((0)/4<<8) + (hue2*3)/4;
        else if(hue2 < 170) {
            hue2 -= 85;
            color = ((0)/4<<16) + ((hue2*3)/4<<8) + (255 - hue2*3)/4;
        }
        else {
            hue2 -= 170;
            color = ((hue2*3)/4<<16) + ((255 - hue2*3)/4<<8) + (0)/4;
        }
        
        lightsUpdate(color);
    }
}
