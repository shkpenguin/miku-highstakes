#include "chassis/odom.h"
#include "chassis/mcl.h"
#include "utils/utils.h"

void initSystem() {
    vertical.reset_position();
	hori.reset_position();

	intake.tare_position();
	lbRot.reset_position();

    optical.set_integration_time(5);

	imu.reset();
	while (imu.is_calibrating()) pros::delay(10);
}

void initAuto(Pose pose) {
    setPose(pose);
    initParticles();
}

void autoSystem() {
    while(true) {
        if(!pros::competition::is_autonomous()) break;
        // Update raw odometry calculations
        updateOdom();
        
        // Update odometry with particle filtering
        updateMCL();

        miku.updateVoltage();
		pros::delay(10);
	}
}