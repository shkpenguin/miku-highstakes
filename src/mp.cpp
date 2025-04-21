#include "mp.h"
#include "utils/utils.h"

#include <iostream>

CubicBezier::CubicBezier(const Point &p0, const Point &p1, const Point &p2, const Point &p3)
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;

    this->lengths = std::vector<double>(this->len + 1, 0);

}

double CubicBezier::get_t_at_arc_length(double arc_length)
{
    // Use binary search to find the t value that corresponds to arc_length
    double target = arc_length;
    int low = 0;
    int high = this->len;
    int mid = 0;
    while (low < high)
    {
        mid = int(low + (high - low) / 2);
        if (this->lengths[mid] < target)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    if (this->lengths[mid] > target)
    {
        mid -= 1;
    }
    double lengthBefore = this->lengths[mid];
    if (lengthBefore == target)
    {
        return mid / double(this->len);
    }
    else
    {
        return (mid + (target - lengthBefore) / (this->lengths[mid + 1] - lengthBefore)) / double(this->len);
    }
}

Point CubicBezier::getPoint(double t) {
    double u = 1 - t;
    double b0 = u * u * u;
    double b1 = 3 * u * u * t;
    double b2 = 3 * u * t * t;
    double b3 = t * t * t;

    double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
    double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
    return Point(x, y);
}

Point CubicBezier::getDerivative(double t)
{
    double u = 1 - t;
    double b0 = -3 * u * u;
    double b1 = 3 * u * u - 6 * u * t;
    double b2 = 6 * u * t - 3 * t * t;
    double b3 = 3 * t * t;

    double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
    double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
    return Point(x, y);
}

Point CubicBezier::getSecondDerivative(double t)
{
    double u = 1 - t;
    double b0 = 6 * u;
    double b1 = -12 * u + 6 * t;
    double b2 = 6 * u - 12 * t;
    double b3 = 6 * t;

    double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
    double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;

    return Point(x, y);
}

double CubicBezier::getCurvature(double t)
{
    Point d = this->getDerivative(t);
    Point dd = this->getSecondDerivative(t);
    double k = (d.x * dd.y - d.y * dd.x) / std::pow(d.x * d.x + d.y * d.y, 1.5);
    return k;
}

double CubicBezier::getCurvature(Point d, Point dd)
{
    double denominator = d.x * d.x + d.y * d.y;
    denominator *= denominator * denominator;
    denominator = std::sqrt(denominator);
    double k = (d.x * dd.y - d.y * dd.x) / denominator;
    return k;
}

ProfilePoint::ProfilePoint(double x, double y, double theta, double curvature, double vel)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->curvature = curvature;
    this->vel = vel;
}

Constraints::Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width)
{
    this->max_vel = max_vel;
    this->max_acc = max_acc;
    this->friction_coef = friction_coef;
    this->max_dec = max_dec;
    this->max_jerk = max_jerk;
    this->track_width = track_width;
}
double Constraints::maxSpeed(double curvature)
{
    double max_turn_speed = ((2 * this->max_vel / this->track_width) * this->max_vel) / (fabs(curvature) * this->max_vel + (2 * this->max_vel / this->track_width));
    if (curvature == 0)
        return max_turn_speed;
    double max_slip_speed = sqrt(this->friction_coef * (1 / abs(curvature)) * 9.81 * 39.3701);
    return std::min(max_slip_speed, max_turn_speed);
}

std::pair<double, double> Constraints::wheelSpeeds(double angularVel, double vel)
{
    double v_left = vel - angularVel * this->track_width / 2;
    double v_right = vel + angularVel * this->track_width / 2;
    return std::make_pair(v_left, v_right);
}

// TrapezoidalProfile::TrapezoidalProfile(Constraints *constraints, double length, double start_vel, double end_vel)
// {
//     this->constraints = constraints;
//     this->length = length;
//     this->start_vel = start_vel;
//     this->end_vel = end_vel;
//     double non_cruise_dist = constraints->max_vel * constraints->max_vel / (2 * constraints->max_acc) + constraints->max_vel * constraints->max_vel / (2 * constraints->max_dec);

//     this->cruise_vel = non_cruise_dist < length ? constraints->max_vel : std::sqrt(2 * (length * constraints->max_acc * constraints->max_dec) / (constraints->max_acc + constraints->max_dec));

//     this->accel_dist = (this->cruise_vel * this->cruise_vel - this->start_vel * this->start_vel) / (2 * constraints->max_acc);
//     // this->decel_dist = length + (end_vel * end_vel - this->cruise_vel * this->cruise_vel) / (2 * constraints->max_dec);
//     this->decel_dist = (cruise_vel * cruise_vel - end_vel * end_vel) / (2 * constraints->max_dec);

//     this->accel_time = (this->cruise_vel - this->start_vel) / constraints->max_acc;
//     this->decel_time = (this->cruise_vel - this->end_vel) / constraints->max_dec;
//     this->cruise_time = (this->length - this->accel_dist - (this->length - this->decel_dist)) / this->cruise_vel;
//     this->total_time = accel_time + cruise_time + decel_time;

// }
// double TrapezoidalProfile::get_vel_at_time(double t)
// {
//     if (t < 0)
//     {
//         t = 0;
//     }
//     if (t > this->total_time)
//     {
//         t = this->total_time;
//     }

//     // If in acceleration phase
//     if (t < this->accel_time)
//     {
//         return this->start_vel + constraints->max_acc * t;
//     }
//     // If in cruising phase
//     else if (t < this->accel_time + this->cruise_time)
//     {
//         return this->cruise_vel;
//     }
//     // If in deceleration phase
//     else
//     {
//         double time_in_decel = t - this->accel_time - this->cruise_time;
//         return this->cruise_vel - constraints->max_dec * time_in_decel;
//     }
// }

ProfileGenerator::ProfileGenerator(Constraints *constraints)
{
    this->constraints = constraints;
}

void ProfileGenerator::generateProfile(CubicBezier *path) {

    this->profile.clear();

    double t = 0.0;
    double dist = 0.0;
    double vel = 0.00001;
    double dt = 0.01;
    Point position = path->getPoint(t);
    double heading;

    std::vector<ProfilePoint> forwardPass;
    forwardPass.push_back(ProfilePoint(position.x, position.y, 0, 0, vel));

    double last_angular_vel;

    while (t <= 1) {
        Point deriv = path->getDerivative(t);
        Point derivSecond = path->getSecondDerivative(t);

        double curvature = path->getCurvature(deriv, derivSecond);
        double angular_vel = vel * curvature;
        double angular_accel = (angular_vel - last_angular_vel) * (vel / dt);
        last_angular_vel = angular_vel;

        double max_accel = this->constraints->max_acc - abs(angular_accel * this->constraints->track_width / 2);

        vel = std::min(this->constraints->maxSpeed(curvature), std::sqrt(vel * vel + 2 * max_accel * dt));
        dist += vel * dt;

        position = path->getPoint(t);
        heading = atan2(deriv.y, deriv.x);
        forwardPass.push_back(ProfilePoint(position.x, position.y, heading, curvature, vel));
        t += dt;
    }

    vel = 0.00001;
    last_angular_vel = 0;
    t = 1;
    int i = forwardPass.size() - 1;

    while (dist >= 0)
    {
        if (i < 0)
        {
            break;
        }
        Point deriv = path->getDerivative(t);
        Point derivSecond = path->getSecondDerivative(t);

        double curvature = path->getCurvature(deriv, derivSecond);
        double angular_vel = vel * curvature;
        double angular_accel = (angular_vel - last_angular_vel) * (vel / dt);
        last_angular_vel = angular_vel;

        double max_accel = this->constraints->max_dec - abs(angular_accel * this->constraints->track_width / 2);
        vel = std::min(this->constraints->maxSpeed(curvature), std::sqrt(vel * vel + 2 * max_accel * dt));
        dist -= vel * dt;

        this->profile.push_back(ProfilePoint(forwardPass[i].x, forwardPass[i].y, forwardPass[i].theta, forwardPass[i].curvature, vel));
        i--;

    }
}

// ChassisSpeeds ProfileGenerator::getProfilePoint(double d)
// {
//     int index = int(d / this->dd);
//     if (index >= this->profile.size())
//     {
//         index = this->profile.size() - 1;
//     }
//     double vel = this->profile[index].vel;
//     double curvature = this->profile[index].curvature;
//     double angular_vel = vel * curvature;
//     double accel = this->profile[index].accel;
//     return ChassisSpeeds(vel, angular_vel, accel, Pose(this->profile[index].x, this->profile[index].y, this->profile[index].theta));
// }