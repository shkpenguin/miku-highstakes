#include <vector>
#include <cmath>
#include <optional>

/*** @brief only numbers!!!!!!!!!! ***/
template <typename T>
int sgn(T x);

/*** @brief only numbers!!!!!!!!!! ***/
template <typename T>
float avg(const std::vector<T>& values);

template <typename T>
float avg(const std::initializer_list<T>& values);

float rad2deg(float rad);

float deg2rad(float deg);

float ema(float newValue, float oldValue, float alpha);

float rot2inch(float circumference, float rot);

float deg2inch(float circumference, float deg);

