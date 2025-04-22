#include "mcl.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

std::vector<Particle> particles;

void motionUpdate(Pose delta) {
    for (auto& p : particles) {
        float dx = delta.x + randomGaussian(0, 0.1);
        float dy = delta.y + randomGaussian(0, 0.1);
        float dtheta = delta.theta + randomGaussian(0, deg2rad(1));

        // Local to global transformation
        float sinT = sin(p.pose.theta);
        float cosT = cos(p.pose.theta);

        float global_dx = dx * cosT - dy * sinT;
        float global_dy = dx * sinT + dy * cosT;

        p.pose.x += global_dx;
        p.pose.y += global_dy;
        p.pose.theta += dtheta;
    }
}

inline float randUniform() {
    return (float)rand() / (float)RAND_MAX;
}

inline float randNormal(float mean = 0.0f, float stddev = 1.0f) {
    static bool hasSpare = false;
    static float spare;

    if (hasSpare) {
        hasSpare = false;
        return mean + stddev * spare;
    }

    hasSpare = true;
    float u, v, s;
    do {
        u = 2.0f * randUniform() - 1.0f;
        v = 2.0f * randUniform() - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = std::sqrt(-2.0f * std::log(s) / s);
    spare = v * s;
    return mean + stddev * u * s;
}


// 1D Gaussian PDF centered at mu=0
inline float gaussianPDF(float error, float sigma) {
    return std::exp(-0.5f * (error*error) / (sigma*sigma)) 
           / (sigma * std::sqrt(2 * M_PI));
}

float randomGaussian(float mean, float stddev) {
    static bool hasSpare = false;
    static float spare;

    if (hasSpare) {
        hasSpare = false;
        return spare * stddev + mean;
    }

    float u1, u2, s;
    do {
        u1 = (float)rand() / RAND_MAX;
        u2 = (float)rand() / RAND_MAX;
    } while (u1 <= 0.00001); // avoid log(0)

    float mag = sqrt(-2.0 * log(u1));
    spare = mag * sin(2 * M_PI * u2);
    hasSpare = true;

    return mag * cos(2 * M_PI * u2) * stddev + mean;
}

Pose sampleAroundPose(const Pose& center, float stddevX, float stddevY, float stddevTheta) {
    float x, y;

    // Keep resampling until within bounds
    do {
        x = randomGaussian(center.x, stddevX);
        y = randomGaussian(center.y, stddevY);
    } while (x < -72 || x > 72 || y < -72 || y > 72);

    float theta = randomGaussian(center.theta, stddevTheta);

    return Pose(x, y, theta);
}

void initParticles() {
    Pose center = getPose(true);  // Make sure you're using radians
    particles.clear();
    particles.reserve(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++) {
        Pose pose = sampleAroundPose(center, 4.0, 4.0, deg2rad(5)); // tweak stddevs as needed
        particles.push_back(Particle{pose, 1.0f / NUM_PARTICLES});
    }
}

const float HALF = 72.0f;
const float INF  = std::numeric_limits<float>::infinity();

constexpr float hx   = 12;  // 12″
constexpr float vy   = 2;    //  2″

/** returns parametric “t” to hit first wall from (ox,oy) in direction θ (math frame) */
float rayDistanceToWall(float ox, float oy, float θ) {
  float dx = std::cos(θ), dy = std::sin(θ);
  float tMin = INF;

  // vertical walls x = ±HALF
  if (std::abs(dx) > 1e-6f) {
    float t1 = (-HALF - ox)/dx;
    if (t1>0) {
      float y1 = oy + t1*dy;
      if (y1>=-HALF && y1<=HALF) tMin = std::min(tMin, t1);
    }
    float t2 = ( HALF - ox)/dx;
    if (t2>0) {
      float y2 = oy + t2*dy;
      if (y2>=-HALF && y2<=HALF) tMin = std::min(tMin, t2);
    }
  }

  // horizontal walls y = ±HALF
  if (std::abs(dy) > 1e-6f) {
    float t3 = (-HALF - oy)/dy;
    if (t3>0) {
      float x3 = ox + t3*dx;
      if (x3>=-HALF && x3<=HALF) tMin = std::min(tMin, t3);
    }
    float t4 = ( HALF - oy)/dy;
    if (t4>0) {
      float x4 = ox + t4*dx;
      if (x4>=-HALF && x4<=HALF) tMin = std::min(tMin, t4);
    }
  }

  return tMin;  // since (dx,dy) is unit length, tMin is actual distance
}

std::vector<float> estimateDistance(const Pose& pose) {
  // 1) compass→math conversion: 0° compass → +Y world → math π/2
  float θm = M_PI_2 - pose.theta;

  // 2) precompute sin/cos
  float c = std::cos(θm), s = std::sin(θm);

  // 3) body‑frame offsets (x_b=forward, y_b=left/right)
  //    left sensor  = (+vy forward, +hx left)
  //    right sensor = (+vy forward, -hx left)
  float lx = pose.x +  vy*c -  hx*s;
  float ly = pose.y +  vy*s +  hx*c;
  float rx = pose.x +  vy*c - (-hx)*s;
  float ry = pose.y +  vy*s + (-hx)*c;

  // 4) firing angles (math frame): left = θm+90°, right = θm–90°
  float lth = θm + M_PI_2;
  float rth = θm - M_PI_2;

  // 5) raycast
  float dL = rayDistanceToWall(lx, ly, lth);
  float dR = rayDistanceToWall(rx, ry, rth);

  // 6) clamp to valid range
  std::vector<float> dist = {-1.0f, -1.0f};
  if (dL <= HALF) dist[0] = dL;
  if (dR <= HALF) dist[1] = dR;
  return dist;
}

void sensorUpdate(const std::vector<float>& actual) {
    // actual[0] = left sensor, actual[1] = right sensor
    const float sigmaLeft  = 2.0f;
    const float sigmaRight = 2.0f;
    const float MAX_ERROR = 5.0f;

    float totalW = 0.0f;

    for (auto& p : particles) {
        auto pred = estimateDistance(p.pose);

        bool useL = (pred[0] != -1 && actual[0] != -1 &&
                     std::abs(pred[0] - actual[0]) <= MAX_ERROR);
        bool useR = (pred[1] != -1 && actual[1] != -1 &&
                     std::abs(pred[1] - actual[1]) <= MAX_ERROR);

        float weight = 1.0f;
        if (useL) {
            float errL = pred[0] - actual[0];
            weight *= gaussianPDF(errL, sigmaLeft);
        }
        if (useR) {
            float errR = pred[1] - actual[1];
            weight *= gaussianPDF(errR, sigmaRight);
        }

        // If neither is valid, default to neutral weight
        if (!useL && !useR) {
            weight = 1e-6f;
        }

        p.weight = weight;
        totalW += weight;
    }

    // Normalize or fallback to uniform if total weight is zero
    if (totalW > 0.0f) {
        for (auto& p : particles) {
            p.weight /= totalW;
        }
    } else {
        float uniform = 1.0f / particles.size();
        for (auto& p : particles) {
            p.weight = uniform;
        }
    }
}

void resampleParticles() {
    std::vector<Particle> newP;
    newP.reserve(particles.size());

    // Bias settings
    const float biasRatio = 0.1f;               // 10% biased particles
    const int numBiased = particles.size() * biasRatio;
    const int numRandom = particles.size() - numBiased;

    // ----------------------------
    // 1. Regular low-variance resampling for (1 - biasRatio) particles
    float invN = 1.0f / particles.size();
    float r = randUniform() * invN;
    float c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < numRandom; m++) {
        float U = r + m * invN;
        while (U > c && i < (int)particles.size() - 1) {
            i++;
            c += particles[i].weight;
        }
        newP.push_back(particles[i]);
        newP.back().weight = invN;  // temp uniform
    }

    // ----------------------------
    // 2. Inject particles near current estimate
    Pose estimate = getEstimateFromParticles();
    for (int i = 0; i < numBiased; i++) {
        Particle p;
        // Add small Gaussian noise around estimate
        p.pose.x = estimate.x + randNormal(0.0f, 1.0f);  // 1 inch std dev
        p.pose.y = estimate.y + randNormal(0.0f, 1.0f);
        p.pose.theta = estimate.theta + randNormal(0.0f, 0.05f);  // ~3 deg
        p.weight = invN;
        newP.push_back(p);
    }

    // Replace old set
    particles.swap(newP);
}

Pose getEstimateFromParticles() {
    float sumWeights = 0.0f;
    float meanX = 0.0f;
    float meanY = 0.0f;
    float sinTheta = 0.0f;
    float cosTheta = 0.0f;

    for (const auto& p : particles) {
        float w = p.weight;
        meanX += p.pose.x * w;
        meanY += p.pose.y * w;
        sinTheta += std::sin(p.pose.theta) * w;
        cosTheta += std::cos(p.pose.theta) * w;
        sumWeights += w;
    }

    if (sumWeights > 0) {
        meanX /= sumWeights;
        meanY /= sumWeights;
        sinTheta /= sumWeights;
        cosTheta /= sumWeights;
    }

    float meanTheta = std::atan2(sinTheta, cosTheta);
    return Pose(meanX, meanY, meanTheta);
}