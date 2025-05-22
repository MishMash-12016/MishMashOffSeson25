package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base.MotorPidBase;

import java.util.Set;

public class MotorPositionProfiledPidSubsystem extends MotorPidBase {


    public MotorPositionProfiledPidSubsystem(String subsystemName) {
        super(subsystemName);
        this.pidController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(0,0));
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    @Override
    public Command holdSetPointCommand(double setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                ((ProfiledPIDController)pidController).reset(getPose(), getVelocity());
                pidController.setSetpoint(setPoint);
            }

            @Override
            public void execute() {
                double pidOutput = pidController.calculate(getPose());
                double feedforwardOutput = 0;


                if (feedforward != null) {
                    feedforwardOutput = feedforward.calculate(((ProfiledPIDController)pidController).getSetpointState().velocity);
                }
                setPower(pidOutput + feedforwardOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(MotorPositionProfiledPidSubsystem.this);
            }
        };
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    @Override
    public Command holdCurrentStateCommand() {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                pidController.reset();
                pidController.setSetpoint(getPose());
            }

            @Override
            public void execute() {
                double pidOutput = pidController.calculate(getPose());
                double feedforwardOutput = 0;


                if (feedforward != null) {
                    feedforwardOutput = feedforward.calculate(pidController.getSetpoint());
                }
                setPower(pidOutput + feedforwardOutput);// apply computed power
            }

            @Override
            public Set<Subsystem> getRequirements() {
                // Declare that this command requires the enclosing subsystem instance
                return Set.of(MotorPositionProfiledPidSubsystem.this);
            }
        };
    }

    /**
     * @param feedforward the feedforward
     */
    public MotorPositionProfiledPidSubsystem withFeedForward(SimpleMotorFeedforward feedforward) {
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
    public MotorPositionProfiledPidSubsystem withFeedforward(double ks, double kv, double ka) {
        return withFeedForward(new SimpleMotorFeedforward(ks, kv, ka));
    }

    /**
     * updates the constraints values
     *
     * @param maxVelocity     the maximum velocity
     * @param maxAcceleration the maximum acceleration
     * @return this subsystem for chaining
     */
    public MotorPositionProfiledPidSubsystem withConstraints(double maxVelocity, double maxAcceleration) {
        ((ProfiledPIDController)pidController).setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        return this;
    }

    /**
     * Sets the acceptable position error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionProfiledPidSubsystem withPositionTolerance(double tolerance) {
        withErrorTolerance(tolerance);
        return this;
    }

    /**
     * Sets the acceptable velocity error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPositionProfiledPidSubsystem withVelocityTolerance(double tolerance) {
        withDerivativeTolerance(tolerance);
        return this;
    }
}
