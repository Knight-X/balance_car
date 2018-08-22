#include <math.h>
#include "ppo-test_weight.hpp"

float tanha(float src) {
    float minus = -src;
    float a = exp(src) - exp(minus);
    float b = exp(src) + exp(minus);
    float res = a / b;
    return res;
}
void fully1(float* a, float* res) {
  for (int i = 0; i < 64; i++) {
         float tmp = 0.0;
      for (int j = 0; j < 8; j++) {
          tmp = tmp + (w1[i * 8 + j] * a[j]);
      }
      res[i] = tmp + b1[i];
  }
}

void relu(float* dst) {
  for (int i = 0; i < 64; i++) {
    dst[i] = fmax(0.0, dst[i]);
  }
}

float fully2(float* a, float res) {
  for (int i = 0; i < 64; i++) {
          res = res + (w2[i] * a[i]);
  }
  float res2 = res + b2[0];
  return res2;
}
float nn(float* input) {
    float middle[64] = {0.0};
    fully1(input, middle);
    relu(middle);
    float res = 0.0;
    res = fully2(middle, res);

    res = tanh(res);

    return res;
}
