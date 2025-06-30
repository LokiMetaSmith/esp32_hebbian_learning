#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

// --- Neural Network Config ---
#define INPUT_NEURONS 6
#define HIDDEN_NEURONS 16
#define OUTPUT_NEURONS 6
#define PRED_NEURONS INPUT_NEURONS

// --- Data Structures ---
typedef struct {
    float weights[HIDDEN_NEURONS][INPUT_NEURONS];
    float hidden_activations[HIDDEN_NEURONS];
    float hidden_bias[HIDDEN_NEURONS];
} HiddenLayer;

typedef struct {
    float weights[OUTPUT_NEURONS][HIDDEN_NEURONS];
    float output_activations[OUTPUT_NEURONS];
    float output_bias[OUTPUT_NEURONS];
} OutputLayer;

typedef struct {
    float weights[PRED_NEURONS][HIDDEN_NEURONS];
    float pred_activations[PRED_NEURONS];
    float pred_bias[PRED_NEURONS];
} PredictionLayer;


#endif // MAIN_H