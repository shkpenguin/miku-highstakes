#include "mcl.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>
#include <numeric>

bool leftEnabled = true;
bool rightEnabled = true;

void enableLeft() {leftEnabled = true;}
void enableRight() {rightEnabled = true;}
void disableLeft() {leftEnabled = false;}
void disableRight() {rightEnabled = false;}

std::vector<Particle> particles;

void motionUpdate(Point delta) {
    for (auto& p : particles) {
        
        // Apply noise to the delta values for each particle to simulate odometry error
        float dx = delta.x + randomGaussian(0, 0.1);  // Reduced noise for x movement
        float dy = delta.y + randomGaussian(0, 0.1);  // Reduced noise for y movement

        // Update particle's position
        p.point.x += dx;
        p.point.y += dy;

        p.point.x = std::clamp(p.point.x, static_cast<double>(-HALF), static_cast<double>(HALF));
        p.point.y = std::clamp(p.point.y, static_cast<double>(-HALF), static_cast<double>(HALF));    


    }
}

float computeESS() {
    float sumSq = 0;
    for (auto& p : particles) {
        sumSq += p.weight * p.weight;
    }
    return (sumSq > 0) ? 1.0f / sumSq : 0.0f;
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

Point sampleAroundPoint(const Point& center, float stddevX, float stddevY) {
    float x, y;

    // Keep resampling until within bounds
    do {
        x = randomGaussian(center.x, stddevX);
        y = randomGaussian(center.y, stddevY);
    } while (x < -72 || x > 72 || y < -72 || y > 72);

    return Point(x, y);
}

void initParticles() {
    Point center = Point(getPose().x, getPose().y);  // Make sure you're using radians
    particles.clear();
    particles.reserve(NUM_PARTICLES);

    for (int i = 0; i < NUM_PARTICLES; i++) {
        Point point = sampleAroundPoint(center, 4.0, 4.0); // tweak stddevs as needed
        particles.push_back(Particle{point, 1.0f / NUM_PARTICLES});
    }
}

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

std::vector<float> estimateDistance(const Point& point) {
  // 1) compass→math conversion: 0° compass → +Y world → math π/2
  float θm = M_PI_2 - getPose(true).theta;

  // 2) precompute sin/cos
  float c = std::cos(θm), s = std::sin(θm);

  // 3) body‑frame offsets (x_b=forward, y_b=left/right)
  //    left sensor  = (+vy forward, +hx left)
  //    right sensor = (+vy forward, -hx left)
  float lx = point.x +  vy*c -  hx*s;
  float ly = point.y +  vy*s +  hx*c;
  float rx = point.x +  vy*c - (-hx)*s;
  float ry = point.y +  vy*s + (-hx)*c;

  // 4) firing angles (math frame): left = θm+90°, right = θm–90°
  float lth = θm + M_PI_2;
  float rth = θm - M_PI_2;

  // 5) raycast
  float dL = rayDistanceToWall(lx, ly, lth);
  float dR = rayDistanceToWall(rx, ry, rth);

  // 6) clamp to valid range
  std::vector<float> dist = {-1.0f, -1.0f};
  if (dL <= HALF && leftEnabled) dist[0] = dL;
  if (dR <= HALF && rightEnabled) dist[1] = dR;
  return dist;
}

Point getEstimate() {
    float sumWeights = 0.0f;
    float meanX = 0.0f;
    float meanY = 0.0f;

    for (const auto& p : particles) {
        float w = p.weight;
        meanX += p.point.x * w;
        meanY += p.point.y * w;
        sumWeights += w;
    }

    if (sumWeights > 0) {
        meanX /= sumWeights;
        meanY /= sumWeights;
    }

    return Point(meanX, meanY);
}

bool sensorUpdate(const std::vector<float>& actual) {
    const float MAX_ERROR = 4.0f;
    float totalW = 0.0f;
    bool anyValid = false;

    for (auto& p : particles) {
        auto pred = estimateDistance(p.point);

        bool useL = (pred[0] != -1 && actual[0] != -1 &&
                     std::abs(pred[0] - actual[0]) <= MAX_ERROR);
        bool useR = (pred[1] != -1 && actual[1] != -1 &&
                     std::abs(pred[1] - actual[1]) <= MAX_ERROR);

        float weight = 1.0f;
        if (useL) {
            weight *= gaussianPDF(pred[0] - actual[0], 1.0f);
            anyValid = true;
        }
        if (useR) {
            weight *= gaussianPDF(pred[1] - actual[1], 1.0f);
            anyValid = true;
        }

        if (!useL && !useR) {
            weight = 1e-6f;
        }

        p.weight = weight;
        totalW += weight;
    }

    if (totalW > 0.0f) {
        for (auto& p : particles) p.weight /= totalW;
    } else {
        float uniform = 1.0f / particles.size();
        for (auto& p : particles) p.weight = uniform;
    }

    return anyValid;
}

void resampleParticles() {
    int N = particles.size();
    std::vector<Particle> newP;
    newP.reserve(N);

    float invN = 1.0f / N;
    float r    = randUniform() * invN;
    float c    = particles[0].weight;
    int   idx  = 0;
    for (int m = 0; m < N; ++m) {
        float U = r + m * invN;
        while (U > c && idx < N-1) {
            idx++;
            c += particles[idx].weight;
        }
        newP.push_back(particles[idx]);
        newP.back().weight = invN;
    }
    particles.swap(newP);
}

void injectAroundEstimate(float essThresholdRatio, float sigma) { 
    int N        = particles.size();
    float ESS    = computeESS();
    float target = essThresholdRatio * N;
    if (ESS >= target) return;  // still plenty of diversity!

    // how many to inject?
    int K = static_cast<int>(target - ESS + 0.5f);
    if (K <= 0) return;

    // find worst‑weighted K indices
    std::vector<int> idx(N);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
              [&](int a, int b){
                  return particles[a].weight < particles[b].weight;
              });

    // get your best estimate
    Point mean = getEstimate();

    // replace the K worst particles
    float uniformW = 1.0f / N;
    for (int i = 0; i < K; ++i) {
        int bad = idx[i];
        particles[bad].point.x     = mean.x     + randNormal(0.0f, sigma);
        particles[bad].point.y     = mean.y     + randNormal(0.0f, sigma);
        particles[bad].weight     = uniformW;
    }
}

void updateMCL() {
    motionUpdate(Point(getSpeed().x, getSpeed().y));

    std::vector<float> sensors = {
        static_cast<float>(leftDist.get() / 25.4),
        static_cast<float>(rightDist.get() / 25.4)
    };

    bool valid = sensorUpdate(sensors);

    if (valid) {
        Pose estPose(getEstimate().x, getEstimate().y, getPose(true).theta);
        setPose(estPose, true);
        resampleParticles();
        injectAroundEstimate(0.5, 5.0);  // Only when we have good data
    }
}