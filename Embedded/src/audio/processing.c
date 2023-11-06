#include "processing.h"
#include "arm_math.h"

#define Blks_Per_Frame 10
#define STEREO_BLOCK_SAMPLES (STEREO_BLOCK_BYTES/CONFIG_AUDIO_BIT_DEPTH_OCTETS)
#define MONO_BLOCK_SAMPLES (STEREO_BLOCK_SAMPLES/2)

#define COEFFICIENT 16


/* Data structures for lms calculation */
static arm_lms_instance_q15 lms_instance;
static q15_t State[MONO_BLOCK_SAMPLES + COEFFICIENT - 1];
static q15_t errorArr[MONO_BLOCK_SAMPLES];
static q15_t coeff_p[COEFFICIENT];

/* local microphone samples buffer for lms block processing */
static sync_buf RX_buf;


void InitBuffer() {
    memset(RX_buf.content, 0, STEREO_BLOCK_BYTES*Buffer_Size);
    RX_buf.blk_count = 0;
}

/* Pushes a block of stereo data to the end of RX_buf buffer. Discard oldest block
   when the buffer is full */
void pushBuffer(int8_t* block_ptr) {
    /* Shift contents in the buffer 1 block forward. Note that memcpy
       has undefined behavior when src and dest location overlaps */
    memmove(RX_buf.content, &RX_buf.content[STEREO_BLOCK_BYTES], STEREO_BLOCK_BYTES * (Buffer_Size-1));
    memcpy(&RX_buf.content[STEREO_BLOCK_BYTES * (Buffer_Size-1)], block_ptr, STEREO_BLOCK_BYTES);
    RX_buf.blk_count++;
    if (RX_buf.blk_count >= Buffer_Size){
        RX_buf.blk_count = Buffer_Size;
    }
}


/* Initilize a FIR filter used for LMS filtering */
void InitFIRFilter(){
    /* Notice that assigning float to q16_t data type directly may cause problems ??? */
    float32_t learningrate_float = 0.0002;
    q15_t learningRate;
    arm_float_to_q15(&learningrate_float, &learningRate, 1);

    /* Assign memory space for performing LMS calculation, set LR and combine them to lms_instance variable */
    arm_lms_init_q15(&lms_instance, COEFFICIENT, coeff_p,  State, learningRate, MONO_BLOCK_SAMPLES, 0);
}

/* Perform LMS filter on one block */
void filterBLK(int16_t* input, int16_t* reference, int16_t* output){
    // Allocate q15_t buffers
    q15_t input_mono[MONO_BLOCK_SAMPLES]; 
    q15_t reference_mono[MONO_BLOCK_SAMPLES];
    q15_t output_mono[MONO_BLOCK_SAMPLES];
    
    /* input and reference has two channels, with samples in each channel stored alternatively, so we take
       samples from the left channel and cast them to q15_t types to prevent overflow 
       int16_t can be cast to q15_t without any conversion. */
    int i;
    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        input_mono[i] = (q15_t) input[2*i];
        reference_mono[i] = (q15_t) reference[2*i];
    }

    arm_lms_q15(&lms_instance, input_mono, reference_mono, output_mono, errorArr, MONO_BLOCK_SAMPLES);

    /* */
    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        output[2*i] = (int16_t)output_mono[i];
    }

}

/* Perform ten blocks LMS with the given reference blocks. Input blocks are taken from RX_buf */
void filterFIR(int16_t* reference, int16_t* output){
    /* Skip processing if no enough blocks in the buffer, wait for next frame */
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

int16_t* getBuffer(){
    return (int16_t*)&RX_buf.content[STEREO_BLOCK_BYTES * (Buffer_Size - RX_buf.blk_count)];
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