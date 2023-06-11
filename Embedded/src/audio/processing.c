#include "processing.h"
#include "arm_math.h"

arm_lms_instance_q15 lms_instance;
q15_t StateF32[AUDIO_BLOCK_SAMPLES*2];
q15_t errorArr[AUDIO_BLOCK_SAMPLES];
q15_t coeff_p[AUDIO_BLOCK_SAMPLES];

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length){
    //convert PCM to float point
    int N = 1;
    float impulse[N];
    int n;
    for (n = 0; n < N; n++) {
        if(n == 0) {
            impulse[n] = 1.0;
        } else {
            impulse[n] = 0.0;        }
    }
    q7_t newimpulse[N];
    arm_float_to_q7(impulse, newimpulse, N);
    arm_conv_q7((q7_t *)decoded_input, decoded_data_length, newimpulse, N, processed_decoded_output);
}

//Initilize a FIR filter used for active noise cancellation
void InitFIRFilter(){
    //declare a arm lms filter
    q15_t learningrate = 0.1;
    arm_lms_init_q15(&lms_instance, AUDIO_BLOCK_SAMPLES, coeff_p,  StateF32, learningrate, AUDIO_BLOCK_SAMPLES, 0);
}

void updataFIRFilter(void* input, void* reference, void* output){
    arm_lms_q15(&lms_instance, (q15_t *)input, (q15_t*)reference, (q15_t*)output, errorArr, AUDIO_BLOCK_SAMPLES);
}
