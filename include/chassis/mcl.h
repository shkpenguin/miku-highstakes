#pragma once

#include "utils/pose.h"
#include "utils/math.h"
#include "utils/utils.h"
#include "chassis/odom.h"
#include "robot-config.h"
#include <vector>

#define NUM_PARTICLES 100

struct Particle {
    Pose pose;
    float weight;

    Particle() : pose(0, 0, 0), weight(1.0f) {}
    Particle(Pose p, float w = 1.0f) : pose(p), weight(w) {}
};

extern std::vector<Particle> particles;

float randomGaussian(float mean, float stddev);
float gaussianPDF(float error, float sigma); // inline
float randUniform(); // inline

Pose sampleAroundPose(const Pose& center, float stddevX, float stddevY, float stddevTheta);
float rayDistanceToWall(float ox, float oy, float θ);
std::vector<float> estimateDistance(const Pose& pose);
void initParticles();
void resampleParticles();
void motionUpdate(Pose delta);
void sensorUpdate(const std::vector<float>);