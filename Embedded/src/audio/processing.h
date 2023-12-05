#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include "arm_math.h"

#define STEREO_BLOCK_BYTES 192
#define STEREO_BLOCK_SAMPLES (STEREO_BLOCK_BYTES/CONFIG_AUDIO_BIT_DEPTH_OCTETS)
#define Buffer_Size 50

typedef struct{
    int16_t data[STEREO_BLOCK_SAMPLES];
} block_1ms_t;

typedef struct{
    block_1ms_t blocks[Buffer_Size];
    size_t blk_count;
} sync_buf;

void InitBuffer();
void pushBuffer(int8_t* block_ptr);
int16_t* getBuffer();

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length);
void InitFIRFilter();
void filterFIR(int16_t* reference, int16_t* output);
void* getCoeffPtr();
void* getErrPtr();

#endif
