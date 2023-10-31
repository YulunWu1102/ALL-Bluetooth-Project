#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include "arm_math.h"

#define STEREO_BLOCK_BYTES 192
#define Blks_Per_Frame 10
#define Buffer_Size 15

typedef struct{
    int8_t content[STEREO_BLOCK_BYTES*Buffer_Size];
    int8_t blk_count;
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
