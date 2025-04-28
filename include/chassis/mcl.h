#pragma once

#include "utils/pose.h"
#include "utils/math.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "robot-config.h"
#include <vector>

#define NUM_PARTICLES 100

const float HALF = 72.0f;
const float INF  = std::numeric_limits<float>::infinity();

constexpr float hx   = 6;  // 6in from center
constexpr float vy   = -1; // 1in forwards of center

struct Particle {
    Point point;
    float weight;

    Particle() : point(0, 0), weight(1.0f) {}
    Particle(Point p, float w = 1.0f) : point(p), weight(w) {}
};

extern std::vector<Particle> particles;

float randomGaussian(float mean, float stddev);
float gaussianPDF(float error, float sigma); // inline
float randUniform(); // inline

Point sampleAroundPoint(const Point& center, float stddevX, float stddevY);
float rayDistanceToWall(float ox, float oy, float θ);
std::vector<float> estimateDistance(const Point& point);
void initParticles();
void resampleParticles();
void motionUpdate(Point delta);
bool sensorUpdate(const std::vector<float>& actual);
Point getEstimate();
void injectAroundEstimate(float essThresholdRatio = 0.5f, float transSigma = 1.0f);

void enableLeft();
void enableRight();
void disableLeft();
void disableRight();

void updateMCL();