#pragma once

#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "utils/pose.h"
#include "utils/pid.h"
#include "utils/exitcondition.h"
#include "api.h"
#include "utils/math.h"
#include "utils/utils.h"

class ControllerSettings {
    public:
        ControllerSettings() 
            : kP(0), kI(0), kD(0), windupRange(0), smallError(0), smallErrorTimeout(0),
              largeError(0), largeErrorTimeout(0), slew(0) {}

        ControllerSettings(float kP, float kI, float kD, float windupRange, float smallError, float smallErrorTimeout,
                           float largeError, float largeErrorTimeout, float slew)
            : kP(kP),
              kI(kI),
              kD(kD),
              windupRange(windupRange),
              smallError(smallError),
              smallErrorTimeout(smallErrorTimeout),
              largeError(largeError),
              largeErrorTimeout(largeErrorTimeout),
              slew(slew) {}

        float kP;
        float kI;
        float kD;
        float windupRange;
        float smallError;
        float smallErrorTimeout;
        float largeError;
        float largeErrorTimeout;
        float slew;
};

struct TurnToPointParams {
        /** whether the robot should turn to face the point with the front of the robot. True by default */
        bool forwards = true;
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        int maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        int minSpeed = 0;
        /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

struct TurnToHeadingParams {
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        int maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the `it conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        int minSpeed = 0;
        /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

enum class DriveSide {
    LEFT, /** lock the left side of the drivetrain */
    RIGHT /** lock the right side of the drivetrain */
};

struct SwingToPointParams {
        /** whether the robot should turn to face the point with the front of the robot. True by default */
        bool forwards = true;
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

struct SwingToHeadingParams {
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can turn at. If set to a non-zero value, the exit conditions will switch to less
         * accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** angle between the robot and target heading where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

struct MoveToPoseParams {
        /** whether the robot should move forwards or backwards. True by default */
        bool forwards = true;
        /** how fast the robot will move around corners. Recommended value 2-15. 0 means use horizontalDrift set in
         * chassis class. 0 by default. */
        float horizontalDrift = 0;
        /** carrot point multiplier. value between 0 and 1. Higher values result in curvier movements. 0.6 by default */
        float lead = 0.6;
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
         * less accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

struct MoveToPointParams {
        /** whether the robot should move forwards or backwards. True by default */
        bool forwards = true;
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
         * less accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

struct MoveDistanceParams {
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
        /** the minimum speed the robot can travel at. If set to a non-zero value, the exit conditions will switch to
         * less accurate but smoother ones. Value between 0-127. 0 by default */
        float minSpeed = 0;
        /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
         * non-zero.*/
        float earlyExitRange = 0;
};

/**
 * @brief Chassis class
 */
class Chassis {
    public:

        //chassis constructor:

        Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
                pros::Rotation* verticalTracker, pros::Rotation* horiTracker, ControllerSettings lateralSettings, 
                ControllerSettings angularSettings);
        
        void setPose(float x, float y, float theta, bool radians = false);

        void setPose(Pose pose, bool radians = false);

        Pose getPose(bool radians = false, bool standardPos = false);

        void waitUntil(float dist);

        void waitUntilDone();

        void setBrakeMode(pros::motor_brake_mode_e mode);

        void turnToPoint(float x, float y, int timeout, TurnToPointParams params = {}, bool async = true);

        void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);

        void swingToHeading(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params = {},
                            bool async = true);

        void swingToPoint(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params = {},
                          bool async = true);

        void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params = {}, bool async = true);

        void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {}, bool async = true);

        void moveDistance(float distance, int timeout, MoveDistanceParams params = {}, bool async = true);

        void moveTime(float time, float speed);

        // void follow(const asset& path, float lookahead, int timeout, bool forwards = true, bool async = true);

        void tank(int left, int right, bool disableDriveCurve = false);

        void arcade(int throttle, int turn, bool disableDriveCurve = false, float desaturateBias = 0.5);

        void curvature(int throttle, int turn, bool disableDriveCurve = false);

        void cancelMotion();

        void cancelAllMotions();

        bool isInMotion() const;

        void resetLocalPosition();

        PID lateralPID;

        PID angularPID;
    protected:

        void requestMotionStart();
        /**
         * @brief Dequeues this motion and permits queued task to run
         */
        void endMotion();

        bool motionRunning = false;
        bool motionQueued = false;

        float distTraveled = 0;

        ControllerSettings lateralSettings;
        ControllerSettings goalSettings;
        ControllerSettings angularSettings;

        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;

        pros::Rotation* verticalTracker;
        pros::Rotation* horiTracker;

        ExitCondition lateralLargeExit;
        ExitCondition lateralSmallExit;
        ExitCondition angularLargeExit;
        ExitCondition angularSmallExit;
    private:
        pros::Mutex mutex;
};