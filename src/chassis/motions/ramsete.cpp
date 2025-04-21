#include "chassis/chassis.h"
#include "mp.h"
#include "utils/timer.h"
#include "chassis/odom.h"
#include "robot-config.h"

double kV = 1.0;
double kW = 2.0;

void Chassis::ramsete(Point p0, Point p1, Point p2, Point p3, int timeout) {
    
    this->requestMotionStart();

    if(!this->motionRunning) return;

    lateralLargeExit.reset();
    lateralSmallExit.reset();

    Timer timer(timeout);

    profileGenerator.generateProfile(new CubicBezier(p0, p1, p2, p3));

    while(!timer.isDone() && this->motionRunning && (!lateralLargeExit.getExit() && !lateralSmallExit.getExit())) {

        double dx = targetPoint.x - getPose().x;
        double dy = targetPoint.y - getPose().y;
        double angle_error = targetPoint.theta - getPose(true, true).theta;

        // Normalizing angle_error to stay within the range [-PI, PI]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Calculate distance and angle errors
        double distance_error = std::sqrt(dx * dx + dy * dy);

        // Use RAMSETE control law
        double v_d = kV * distance_error;
        double omega_d = kW * angle_error;

        // Calculate left and right wheel speeds
        double left_speed = v_d - (omega_d * TRACK_WIDTH / 2);
        double right_speed = v_d + (omega_d * TRACK_WIDTH / 2);

    }

}