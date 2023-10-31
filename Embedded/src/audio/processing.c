#include "processing.h"
#include "arm_math.h"


#define MONO_BLOCK_BYTES (STEREO_BLOCK_BYTES/2)
#define BYTE_DEPTH 2
#define MONO_BLOCK_SAMPLES (MONO_BLOCK_BYTES/BYTE_DEPTH)
#define STEREO_BLOCK_SAMPLES (STEREO_BLOCK_BYTES/BYTE_DEPTH)

#define COEFFICIENT 16

static arm_lms_instance_q15 lms_instance;
static q15_t State[MONO_BLOCK_SAMPLES + COEFFICIENT - 1];
static q15_t errorArr[MONO_BLOCK_SAMPLES];
static q15_t coeff_p[COEFFICIENT];

static sync_buf RX_buf;

void InitBuffer() {
    memset(RX_buf.content, 0, STEREO_BLOCK_BYTES*Buffer_Size);
    RX_buf.blk_count = 0;
}

void pushBuffer(int8_t* block_ptr) {
    memmove(RX_buf.content, &RX_buf.content[STEREO_BLOCK_BYTES], STEREO_BLOCK_BYTES * (Buffer_Size-1));
    memcpy(&RX_buf.content[STEREO_BLOCK_BYTES * (Buffer_Size-1)], block_ptr, STEREO_BLOCK_BYTES);
    RX_buf.blk_count++;
    if (RX_buf.blk_count >= Buffer_Size){
        RX_buf.blk_count = Buffer_Size;
    }
}

int16_t* getBuffer(){
    return (int16_t*)&RX_buf.content[STEREO_BLOCK_BYTES * (Buffer_Size - RX_buf.blk_count)];
}

void retriveBlocks(int8_t* targetPacket){
    if (RX_buf.blk_count >= Blks_Per_Frame){
        memcpy(targetPacket, &RX_buf.content[(RX_buf.blk_count-Blks_Per_Frame) * STEREO_BLOCK_BYTES], STEREO_BLOCK_BYTES*Blks_Per_Frame);
        RX_buf.blk_count -= Blks_Per_Frame;   
    }
}

//Initilize a FIR filter used for LMS filtering
void InitFIRFilter(){
    float32_t learningrate_float = 0.0002;
    q15_t learningRate;
    arm_float_to_q15(&learningrate_float, &learningRate, 1);
    arm_lms_init_q15(&lms_instance, COEFFICIENT, coeff_p,  State, learningRate, MONO_BLOCK_SAMPLES, 0);
}

void filterBLK(int16_t* input, int16_t* reference, int16_t* output){
    // Allocate q15_t buffers
    // update 9/26/2023: change all to float32_t
    q15_t input_mono[MONO_BLOCK_SAMPLES]; 
    q15_t reference_mono[MONO_BLOCK_SAMPLES];
    q15_t output_mono[MONO_BLOCK_SAMPLES];
    
    // input has one channel, so we take the last 48 samples
    // reference has two channels, with samples in each channel stored alternatively, so we take
    // 48 samples from the left channel
    int i;
    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        input_mono[i] = (q15_t) input[2*i];
        reference_mono[i] = (q15_t) reference[2*i];
    }

    arm_lms_q15(&lms_instance, input_mono, reference_mono, output_mono, errorArr, MONO_BLOCK_SAMPLES);

    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        output[2*i] = (int16_t)output_mono[i];
    }

}

void filterFIR(int16_t* reference, int16_t* output){
    if (RX_buf.blk_count >= Blks_Per_Frame){
        int blk;
        int startIdx = RX_buf.blk_count - Blks_Per_Frame;
        for (blk = 0; blk < Blks_Per_Frame; blk++){
            filterBLK((int16_t*)&RX_buf.content[(startIdx + blk)*STEREO_BLOCK_BYTES], &reference[blk*STEREO_BLOCK_SAMPLES], &output[blk*STEREO_BLOCK_SAMPLES]);
        }
        RX_buf.blk_count -= Blks_Per_Frame;
    }
}

// Functions for initial testing and debugging
void* getCoeffPtr(){
    return coeff_p;
}

void* getErrPtr(){
    return errorArr;
}

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