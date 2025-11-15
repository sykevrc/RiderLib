#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

inline float sinc(float x) {
    if (fabsf(x) < 1e-6f) return 1.0f;
    return sinf(x) / x;
}

void lemlib::Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async) {
    // Take mutex
    this->requestMotionStart();
    if (!this->motionRunning) return;

    if (async) {
        pros::Task task([&]() { moveToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    // Reset exit conditions and PIDs
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);

    // Ramsete tuning
    const float b = 1.0f;       // aggressiveness
    const float zeta = .5f;    // damping
    const float wheelbase = drivetrain.trackWidth;
    const float v_max = params.maxSpeed;
    const float w_max = 5.0f;

    // Trapezoidal velocity profile params
    const float t_ramp_frac = 0.1f;  // fraction of path for accel/decel

    // Main loop
    while (!timer.isDone() && this->motionRunning) {
        Pose pose = getPose(true, true);
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        float distTarget = pose.distance(target);
        float progress = std::clamp(distTraveled / distTarget, 0.0f, 1.0f);

        // --- Trapezoidal feedforward ---
        float v_ff;
        if (progress < t_ramp_frac)
            v_ff = v_max * (progress / t_ramp_frac);
        else if (progress > 1.0f - t_ramp_frac)
            v_ff = v_max * ((1.0f - progress) / t_ramp_frac);
        else
            v_ff = v_max;

        // --- Compute errors ---
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float e_theta = atan2f(sinf(target.theta - pose.theta), cosf(target.theta - pose.theta));

        float lookahead = 5.0f; // mm or inches
        dx -= lookahead * cos(target.theta);
        dy -= lookahead * sin(target.theta);

        float e_x = cos(pose.theta) * dx + sin(pose.theta) * dy;
        float e_y = -sin(pose.theta) * dx + cos(pose.theta) * dy;

        // Ramsete gains
        float k = 2.0f * zeta * sqrtf(b * v_ff * v_ff + w_max * w_max);
        float lateralGain = 1.5f;  // stronger lateral correction
        float v_cmd = v_ff * cos(e_theta) + k * e_x;
        float w_cmd = b * lateralGain * v_ff * sinc(e_theta) * e_y + k * e_theta;

        // Dynamic forward scaling based on lateral error
        float maxLat = 10.0f; // adjust as needed
        float speedScale = std::clamp(1.0f - fabsf(e_y) / maxLat, 0.3f, 1.0f);

        v_cmd *= speedScale;
        // --- Apply feedforward deceleration taper near target ---
        float slowZone = 12.0f; // distance units
        float slowMult = std::clamp(distTarget / slowZone, 0.15f, 1.0f);
        v_cmd *= slowMult;
        w_cmd *= slowMult;

        // --- Clamp velocities ---
        v_cmd = std::clamp(v_cmd, -v_max, v_max);
        w_cmd = std::clamp(w_cmd, -w_max, w_max);

        // --- Convert to wheel velocities ---
        float v_L = v_cmd - w_cmd * wheelbase / 2.0f;
        float v_R = v_cmd + w_cmd * wheelbase / 2.0f;

        float leftPower = std::clamp(v_L, -v_max, v_max);
        float rightPower = std::clamp(v_R, -v_max, v_max);

        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // --- Deadzone stop threshold ---
        if (distTarget < 1.0 && fabs(v_cmd) < 1.0 && fabs(w_cmd) < 0.2) {
            drivetrain.leftMotors->move(0);
            drivetrain.rightMotors->move(0);
            break;
        }

        pros::delay(10);
    }

    // Motion finished
    distTraveled = -1;
    this->endMotion();
}
