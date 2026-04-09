#ifndef GPT_MODEL_H
#define GPT_MODEL_H

#define T_MAX  10
#define N_U     6
#define N_Y     1

void gpt_forward(const float input[T_MAX * N_U], float output[T_MAX * N_Y]);

#endif /* GPT_MODEL_H */