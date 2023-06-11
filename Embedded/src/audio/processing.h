#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include "arm_math.h"

#define AUDIO_BLOCK_SAMPLES 192/2

void TestLibrary(uint8_t * decoded_input, uint8_t * processed_decoded_output, size_t decoded_data_length);
void InitFIRFilter();
void updataFIRFilter(void* input, void* reference, void* output);

#endif
