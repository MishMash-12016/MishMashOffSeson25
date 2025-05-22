package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.MotorPidBase;



//TODO: add manual logging using the subsystemName
public class MotorPositionPidSubsystem extends MotorPidBase {

    public MotorPositionPidSubsystem(String subsystemName){
        super(subsystemName);
    }


    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withPositionTolerance(double tolerance) {
        withErrorTolerance(tolerance);
        return this;
    }

    /**
     * Sets the acceptable velocity error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionPidSubsystem withVelocityTolerance(double tolerance) {
        withDerivativeTolerance(tolerance);
        return this;
    }
}
