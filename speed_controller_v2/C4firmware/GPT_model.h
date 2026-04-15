#ifndef GPT_MODEL_H
#define GPT_MODEL_H

#define W_MAX       10
#define N_LAYER     1
#define N_HEAD      2
#define N_EMBD      12
#define N_U         6
#define N_Y         1
#define N_HEAD_SIZE 2
#define N_EMBD3     36
#define N_EMBD4     48

void gpt_forward(const float *input, float *output);

#endif /* GPT_MODEL_H */