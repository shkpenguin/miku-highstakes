#pragma once

#include "utils/pose.h"

class CubicBezier
{
public:
    CubicBezier(const Point &p0, const Point &p1, const Point &p2, const Point &p3);
    ~CubicBezier()
    {
        this->lengths.clear();
    }

    Point getPoint(double t);
    Point getDerivative(double t);
    Point getSecondDerivative(double t);
    double get_t_at_arc_length(double arc_length);
    double getLength();
    double getCurvature(double t);
    double getCurvature(Point d, Point dd);

private:
    Point p0;
    Point p1;
    Point p2;
    Point p3;

    std::vector<double> lengths;

    double len;
    double curvature;
    int length_samples;
};

class Constraints
{
public:
    Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width);
    double maxSpeed(double curvature);
    std::pair<double, double> wheelSpeeds(double angularVel, double vel);
    double max_vel;
    double max_acc;
    double friction_coef;
    double max_dec;
    double max_jerk;
    double track_width;
};

class ProfilePoint
{
public:
    ProfilePoint(double x, double y, double theta, double curvature, double vel);
    double x;
    double y;
    double theta;
    double curvature;
    double vel;
};

class ChassisSpeeds
{
public:
    ChassisSpeeds(double vel, double omega, double accel, Pose pose) {
        this->vel = vel;
        this->omega = omega;
        this->accel = accel;
        this->pose = pose;
    }
    double vel;
    double omega;
    double accel;
    Pose pose;
};

class ProfileGenerator
{
public:
    ProfileGenerator(Constraints *constraints);
    void generateProfile(CubicBezier *path);
    ChassisSpeeds getProfilePoint(double d);
    auto getProfile() { return profile; }

private:
    Constraints *constraints;
    std::vector<ProfilePoint> profile;
    double duration;
};

// class TrapezoidalProfile {
// public:
//     TrapezoidalProfile(Constraints *constraints, double length, double start_vel = 0, double end_vel = 0);
//     double get_vel_at_time(double time);

// private:
//     Constraints *constraints;
//     double length;
//     double start_vel;
//     double end_vel;
//     double cruise_vel;
//     double accel_dist;
//     double decel_dist;
//     double accel_time;
//     double decel_time;
//     double cruise_time;
//     double total_time;
// };