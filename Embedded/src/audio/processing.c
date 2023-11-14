#include "processing.h"
#include "stdint.h"
#include "arm_math.h"

#define Blks_Per_Frame 10
#define MONO_BLOCK_SAMPLES (STEREO_BLOCK_SAMPLES/2)

#define COEFFICIENT 16
#define Coeff_size STEREO_BLOCK_SAMPLES

/* Data structures for lms calculation */
static arm_lms_instance_q15 lms_instance;
static q15_t State[MONO_BLOCK_SAMPLES + COEFFICIENT - 1];
static q15_t errorArr[MONO_BLOCK_SAMPLES];
static q15_t coeff_p[COEFFICIENT];

static float prev_audio_buffer[Coeff_size-1];
static float coeff_arr[Coeff_size];
static float error_arr[Coeff_size];


/* local microphone samples buffer for lms block processing */
static sync_buf RX_buf;


void InitBuffer() {
    memset( (void*) RX_buf.blocks, 0, sizeof(block_1ms_t)*Buffer_Size);
    RX_buf.blk_count = 0;
}

/* Pushes a block of stereo data to the end of RX_buf buffer. Discard oldest block
   when the buffer is full */
void pushBuffer(int8_t* block_ptr) {
    /* Shift contents in the buffer 1 block forward. Note that memcpy
       has undefined behavior when src and dest location overlaps */
    memmove( (void*) &RX_buf.blocks[0], (void*) &RX_buf.blocks[1], sizeof(block_1ms_t) * (Buffer_Size-1));
    memcpy( (void*) &RX_buf.blocks[Buffer_Size-1], block_ptr, sizeof(block_1ms_t));
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

    int i;
    for (i = 0; i < Coeff_size-1; i++){
        prev_audio_buffer[i] = 0.0;
    }

    for (i = 0; i < Coeff_size; i++){
        coeff_arr[i] = 0.0;
        error_arr[i] = 0.0;
    }
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
    

    /* Moving data back to output buffer */
    for (i = 0; i < MONO_BLOCK_SAMPLES; i++){
        output[2*i] = (int16_t)output_mono[i];
        // output[2*i + 1] = 0;
    }
}

/* Perform ten blocks LMS with the given reference blocks. Input blocks are taken from RX_buf */
void filterFIR(int16_t* reference, int16_t* output){
    // /* Skip processing if no enough blocks in the buffer, wait for next frame */
    // if (RX_buf.blk_count >= Blks_Per_Frame){
    //     int blk; 
    //     int startIdx = Buffer_Size - RX_buf.blk_count;
    //     for (blk = 0; blk < Blks_Per_Frame; blk++){
    //         filterBLK(RX_buf.blocks[startIdx + blk].data, &reference[blk*STEREO_BLOCK_SAMPLES], &output[blk*STEREO_BLOCK_SAMPLES]);
    //     }
    //     RX_buf.blk_count -= Blks_Per_Frame;
    // }

    neo_loa_stream_cgd(RX_buf.blocks, reference);

}

void LMS(float input[Coeff_size][Coeff_size], float reference[Coeff_size]){
    float pred[Coeff_size];
    int i;
    for(i = 0; i < Coeff_size; i++){
        arm_dot_prod_f32(input, coeff_arr, Coeff_size, &pred[i]);
    }
    
    for (i = 0; i < Coeff_size; i++){
        error_arr[i] = reference[i] - pred[i];
    }

    float step[Coeff_size];
    for(i = 0; i < Coeff_size; i++){
        arm_dot_prod_f32(input, error_arr, Coeff_size, &step[i]);
    }

    for(i = 0; i < Coeff_size; i++){
        coeff_arr[i] += 0.001 * step[i];
    }
}



void neo_ola_calc_cgd(float* input_buffer, float* reference_buffer){
    float buffer_temp[Coeff_size-1 + Coeff_size];
    memcpy(buffer_temp, prev_audio_buffer, sizeof(float)*(Coeff_size-1));
    memcpy(&buffer_temp[Coeff_size-1], input_buffer, sizeof(float)*Coeff_size);

    float audio_matrix[Coeff_size][Coeff_size];
    fill_matrix(buffer_temp, audio_matrix);

    int i;
    for (i = 0; i < Coeff_size; i++)
    {
        arm_dot_prod_f32(audio_matrix[i], coeff_arr, Coeff_size, &input_buffer[i]);
    }

    LMS(input_buffer, reference_buffer);    
}

void fill_matrix(float* audio_buffer, float allocated_matrix[Coeff_size][Coeff_size]){
    int row = 0;
    for (row = 0; row < Coeff_size; row++){
        memcpy((void*)allocated_matrix[row], &audio_buffer[row], sizeof(float)*Coeff_size);
    }
}

void neo_loa_stream_cgd(float* input, float* reference){
    int blk_count = STEREO_BLOCK_SAMPLES * Blks_Per_Frame / Coeff_size;
    int i;
    float input_blk_buffer[STEREO_BLOCK_SAMPLES], reference_blk_buffer[STEREO_BLOCK_SAMPLES];


    for (i = 0; i < blk_count; i++){
        neo_ola_calc_cgd(&input[STEREO_BLOCK_SAMPLES * i], &reference[STEREO_BLOCK_SAMPLES * i]);
    }
}

void transposeMatrix(float* src, float* dst, int srcRows, int srcCols) {
    for (int i = 0; i < srcRows; i++) {
        for (int j = 0; j < srcCols; j++) {
            dst[j * srcRows + i] = src[i * srcCols + j];
        }
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
    return RX_buf.blocks[Buffer_Size-RX_buf.blk_count].data;
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


// #include <stdio.h>
// #include <gsl/gsl_matrix.h>
// #include <gsl/gsl_vector.h>
// #include <gsl/gsl_linalg.h>

// int main() {
//     // Define the size of the matrix A (n x n)
//     size_t n = 3;

//     // Create matrix A and vectors x, b
//     gsl_matrix *A = gsl_matrix_alloc(n, n);
//     gsl_vector *x = gsl_vector_alloc(n);
//     gsl_vector *b = gsl_vector_alloc(n);

//     // Initialize matrix A and vector b with some values
//     for (size_t i = 0; i < n; ++i) {
//         for (size_t j = 0; j < n; ++j) {
//             gsl_matrix_set(A, i, j, i + j + 1); // Example values for A
//         }
//         gsl_vector_set(b, i, i + 1); // Example values for b
//     }

//     // Create a permutation matrix p to track the permutation
//     gsl_permutation *p = gsl_permutation_alloc(n);

//     // Perform LU decomposition of matrix A
//     int signum; // will store the sign of the permutation
//     gsl_linalg_LU_decomp(A, p, &signum);

//     // Solve the system Ax = b
//     gsl_linalg_LU_solve(A, p, b, x);

//     // Print the solution vector x
//     printf("Solution vector x:\n");
//     for (size_t i = 0; i < n; ++i) {
//         printf("%g\n", gsl_vector_get(x, i));
//     }

//     // Free allocated memory
//     gsl_matrix_free(A);
//     gsl_vector_free(x);
//     gsl_vector_free(b);
//     gsl_permutation_free(p);

//     return 0;
// }

// void CGD(float *A_data, float *x_data, float *b_data, int num_iter, float lr, int size) {
//     arm_matrix_instance_f32 A, x, b, temp, result;
//     float r[size * size]; // Temporary storage for intermediate results
//     float result_data[size];      // Storage for the result of matrix multiplication

//     // Initialize matrix and vector instances
//     arm_mat_init_f32(&A, size, size, A_data);
//     arm_mat_init_f32(&x, size, 1, x_data);
//     arm_mat_init_f32(&b, size, 1, b_data);
//     arm_mat_init_f32(&r, size, size, temp_data);
//     arm_mat_init_f32(&result, size, 1, result_data);

//     // Example matrix-vector multiplication: result = A * x
//     arm_mat_mult_f32(&A, &x, &result);

//     // Now 'result' contains the result of the matrix multiplication A * x
//     // Continue with the rest of the CGD algorithm...

// }
