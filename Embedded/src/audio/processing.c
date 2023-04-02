#include "processing.h"

void TestLibrary(void* decoded_input, void* processed_decoded_output, size_t decoded_data_length){
    int N = 10;
    q15_t impulse[N];
    int n;
    for (n = 0; n < N; n++) {
        if(n == 0) {
            impulse[n] = 1;
        } else {
            impulse[n] = 0;        }
    }
    arm_conv_fast_q15((q15_t *)decoded_input, decoded_data_length/2, (q15_t *)&impulse, n, processed_decoded_output);
    
}
