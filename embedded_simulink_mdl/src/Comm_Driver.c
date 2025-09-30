#include "Comm_Driver.h"
#include <stdio.h>  
#include <string.h> 
void Comm_Driver_Init(void) {
    // no implementation yet
}
float Comm_Driver_Step(float eulerAngles[3], float bodyRates[3], int pos[2], char txBuffer[COMM_TX_LEN]) {
    txBuffer[0] = 0xAA;                      /* sync byte */

    /* pack as (little-endian) 8 × float32 */
    memcpy(txBuffer+1,  eulerAngles,   3*sizeof(float));
    memcpy(txBuffer+13, bodyRates,   3*sizeof(float));
    memcpy(txBuffer+25, pos, 2*sizeof(int));

    return eulerAngles[1];
}

void Comm_Driver_Terminate(void) {
    // not necessary
}
