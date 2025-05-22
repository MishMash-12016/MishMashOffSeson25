package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Velocity;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.MotorPidBase;

/**
 * MotorVelocityPidSubsystem provides PID-controlled velocity for a motor-driven mechanism.
 *
 * <p>This subsystem uses a CuttleEncoder to measure velocity, a list of CuttleMotors to apply power,
 * and a PIDController to compute the required power to reach a target velocity setpoint.
 * </p>
 */
//TODO: add manual logging using the subsystemName
public class MotorVelocityPidSubsystem extends MotorPidBase {


    public MotorVelocityPidSubsystem(String subsystemName) {
        super(subsystemName);
    }

    /**
     * @param feedforward the feedforward
     */
    public MotorVelocityPidSubsystem withFeedForward(SimpleMotorFeedforward feedforward) {
        this.feedforward = feedforward;
        return this;
    }

    /**
     * Updates feedforward gains.
     *
     * @param ks static gain
     * @param kv velocity gain
     * @param ka acceleration gain
     */
    public MotorVelocityPidSubsystem withFeedforward(double ks, double kv, double ka) {
        return withFeedForward(new SimpleMotorFeedforward(ks, kv, ka));
    }

    /**
     * Sets tolerance for velocity error.
     */
    public MotorVelocityPidSubsystem withVelocityTolerance(double tolerance) {
        withErrorTolerance(tolerance);
        return this;
    }

    public MotorVelocityPidSubsystem withAccelerationTolerance(double tolerance) {
        withDerivativeTolerance(tolerance);
        return this;
    }
}
