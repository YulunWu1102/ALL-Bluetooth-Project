#include "processing.h"
#include "arm_math.h"


#define AUDIO_BLOCK_SAMPLES 96
#define COEFFICIENT 6

arm_lms_instance_q15 lms_instance;
q15_t State[AUDIO_BLOCK_SAMPLES + COEFFICIENT - 1];
q15_t errorArr[AUDIO_BLOCK_SAMPLES];
float32_t coeff_p[COEFFICIENT];
//output
float y_predict_result[AUDIO_BLOCK_SAMPLES];
int16_t y_predict_result_converted[AUDIO_BLOCK_SAMPLES];

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length){
    //convert PCM to float points
    int N = 1;
    float impulse[N];
    int n;
    for (n = 0; n < N; n++) {
        if(n == 0) {
            impulse[n] = 1.0;
        } else {
            impulse[n] = 0.0;
        }
    }
    q7_t newimpulse[N];
    arm_float_to_q7(impulse, newimpulse, N);
    arm_conv_q7((q7_t *)decoded_input, decoded_data_length, newimpulse, N, processed_decoded_output);
}

//Initilize a FIR filter used for LMS filtering
void InitFIRFilter(){
    //Adaptive filter coefficients array
    for(int i = 0; i < COEFFICIENT; i++) {
        coeff_p[i] = 0.0;
    }
    
    //declare a arm lms filter
    //q15_t learningrate = 0.1;
    //arm_lms_init_q15(&lms_instance, COEFFICIENT, coeff_p,  State, learningrate, AUDIO_BLOCK_SAMPLES, 0);
}

void filterFIR(uint32_t* input, void* reference, void* output){
    arm_lms_q15(&lms_instance, (q15_t *)input, (q15_t*)reference, (q15_t*)output, errorArr, AUDIO_BLOCK_SAMPLES);
}

void* getCoeffPtr(){
    return errorArr;
}

//def lms(x, y, K, mu = 0.01):
void customFIR(uint32_t* input, uint8_t* reference){
    float mu = 0.005; 
    //Adaptive filter input array Vector to store [x[n],x[n-1],...,x[n-K+1]]
    float xn[COEFFICIENT];
    int i;
    for(i = 0; i < COEFFICIENT; i++) {
        xn[i] = 0.0;
    }
    //Vector to track filtering error
    float y_err[AUDIO_BLOCK_SAMPLES];
    for(i = 0; i < COEFFICIENT; i++) {
        y_err[i] = 0.0;
    }
    //dot product -- real time system
    for(i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        //Update most recent (current) value
        xn[0] = input[AUDIO_BLOCK_SAMPLES - i];
        //y_predict = np.dot(xn, coeff_p)
        //y_predict_result.append(y_predict);
        arm_dot_prod_f32(xn, coeff_p, COEFFICIENT, &y_predict_result[i]);
        float e = y_predict_result[i] - (int16_t)reference[i]; 
        int k;
        for (k = 0; k < COEFFICIENT; k++){
            coeff_p[k] = coeff_p[k] - mu * e * xn[k];
        }
        //xn[k] = np.roll(xn, 1);
        for (k = COEFFICIENT - 1; k > 0; k--){
            xn[k] = xn[k - 1];
        }
        y_err[i] = e;
    }
}

int16_t* getResult(){
    int i;
    for (i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        y_predict_result_converted[i] = (int16_t) y_predict_result[i];
    }
    return y_predict_result_converted;
}