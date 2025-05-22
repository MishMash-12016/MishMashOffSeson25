package org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Base;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.PIDController;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MotorPidBase extends MotorSubsystem{
    // Encoder that measures current position and velocity (ticks converted via ratio)
    private CuttleEncoder encoder;
    public PIDController pidController = new PIDController(0, 0, 0);
    public SimpleMotorFeedforward feedforward;

    //base
    public MotorPidBase(String subsystemName) {
        super(subsystemName);
    }

    /**
     * Creates a Command that moves the mechanism to the specified setpoint using PID control.
     *
     * @param setPoint target setpoint (in encoder units, adjusted by ratio)
     * @return a Command requiring this subsystem
     */
    public Command getToSetpointCommand(double setPoint) {
        return holdSetPointCommand(setPoint).interruptOn(pidController::atSetpoint);
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
    public Command holdSetPointCommand(double setPoint) {
        return new Command() {
            @Override
            public void initialize() {
                // clear previous errors/integral
                pidController.reset();
                pidController.setSetpoint(setPoint);
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
                return Set.of(MotorPidBase.this);
            }
        };
    }

    /**
     * Creates a Command that keeps the mechanism in place using PID control.
     *
     * @return a Command requiring this subsystem
     */
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
                return Set.of(MotorPidBase.this);
            }
        };
    }

    /**
     * Returns the current position (pose) provided by the encoder.
     *
     * @return current pose in encoder units (divided by ratio)
     */
    public double getPose() {
        return encoder.getPose();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public MotorPidBase withEncoder(CuttleEncoder encoder) {
        this.encoder = encoder;
        return this;
    }

    public MotorPidBase withEncoder(CuttleRevHub revHub, int encoderPort, double cpr, Direction direction) {
        return withEncoder(new CuttleEncoder(revHub, encoderPort, cpr, direction));
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     *
     * @param zeroSwitch digital switch input
     * @param zeroPose   encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public MotorPidBase withZeroSwitch(CuttleDigital zeroSwitch, double zeroPose) {
        new Trigger(zeroSwitch::getState)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    /**
     * Configures a zero-position limit switch that resets encoder when activated.
     *
     * @param zeroSupplier a supplier of when to zero the system
     * @param zeroPose     encoder value to set when switch is pressed
     * @return this subsystem for chaining
     */
    public MotorPidBase withZeroSupplier(BooleanSupplier zeroSupplier, double zeroPose) {
        new Trigger(zeroSupplier)
                .whenActive(() -> encoder.setPose(zeroPose));
        return this;
    }

    /**
     * Updates PID gains.
     *
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     * @return this subsystem for chaining
     */
    public MotorPidBase withPid(double kp, double ki, double kd) {
        pidController.setPID(kp, ki, kd);
        return this;
    }

    public MotorPidBase withPid(PIDController pidController) {
        this.pidController = pidController;
        return this;
    }



    /**
     * Sets the acceptable error tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPidBase withErrorTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
        return this;
    }

    /**
     * Sets the acceptable derivative tolerance for the PID setpoint.
     *
     * @param tolerance allowable error range
     * @return this subsystem for chaining
     */
    public MotorPidBase withDerivativeTolerance(double tolerance) {
        pidController.setTolerance(pidController.getErrorTolerance(), tolerance);
        return this;
    }

    /**
     * Defines the Integral Zone (I-Zone) for the PID controller.
     *
     * @param iZone range around setpoint where integral is active
     * @return this subsystem for chaining
     */
    public MotorPidBase withIZone(double iZone) {
        pidController.setIZone(iZone);
        return this;
    }

    /**
     * Restricts the integral accumulator within given bounds.
     *
     * @param minIntegralRange minimum accumulator value
     * @param maxIntegralRange maximum accumulator value
     * @return this subsystem for chaining
     */
    public MotorPidBase withIntegralRange(double minIntegralRange, double maxIntegralRange) {
        pidController.setIntegratorRange(minIntegralRange, maxIntegralRange);
        return this;
    }


    private DoubleSupplier debugKpSupplier;
    private DoubleSupplier debugKiSupplier;
    private DoubleSupplier debugKdSupplier;
    private DoubleSupplier debugIZoneSupplier;
    private DoubleSupplier debugPoseToleranceSupplier;
    private DoubleSupplier debugVelocityToleranceSupplier;

    /**
     * add suppliers that when changed will auto update the pid values.
     * any value you dont need just put null
     *
     * @param debugKpSupplier            Kp
     * @param debugKiSupplier            Kd
     * @param debugKdSupplier            Ki
     * @param debugIZoneSupplier         iZone
     * @param debugPoseToleranceSupplier position tolerance
     * @implNote !NOTICE THIS ONLY WORKS IF IN DEBUG MODE
     */
    public MotorPidBase withDebugPidSuppliers(DoubleSupplier debugKpSupplier,
                                                           DoubleSupplier debugKiSupplier,
                                                           DoubleSupplier debugKdSupplier,
                                                           DoubleSupplier debugIZoneSupplier,
                                                           DoubleSupplier debugPoseToleranceSupplier,
                                                           DoubleSupplier debugVelocityToleranceSupplier) {

        this.debugKpSupplier = debugKpSupplier;
        this.debugKiSupplier = debugKiSupplier;
        this.debugKdSupplier = debugKdSupplier;
        this.debugIZoneSupplier = debugIZoneSupplier;
        this.debugPoseToleranceSupplier = debugPoseToleranceSupplier;

        return this;
    }

    @Override
    public void periodic() {

        //if the current op mode is in debug mode the pid values will auto update from the debug suppliers
        if (MMRobot.getInstance().currentOpMode != null &&
                MMRobot.getInstance().currentOpMode.opModeType == OpModeType.NonCompetition.DEBUG) {
            if (debugKpSupplier != null &&
                    pidController.getP() != debugKpSupplier.getAsDouble()){

                pidController.setP(debugKpSupplier.getAsDouble());
            }

            if (debugKiSupplier != null &&
                    pidController.getI() != debugKiSupplier.getAsDouble()) {

                pidController.setI(debugKiSupplier.getAsDouble());
            }

            if (debugKdSupplier != null &&
                    pidController.getD() != debugKdSupplier.getAsDouble()) {

                pidController.setD(debugKdSupplier.getAsDouble());
            }

            if (debugIZoneSupplier != null &&
                    pidController.getIZone() != debugIZoneSupplier.getAsDouble()) {

                pidController.setIZone(debugIZoneSupplier.getAsDouble());
            }

            if (debugPoseToleranceSupplier != null &&
                    pidController.getErrorTolerance() != debugPoseToleranceSupplier.getAsDouble()) {

                withErrorTolerance(debugPoseToleranceSupplier.getAsDouble());
            }

            if (debugVelocityToleranceSupplier != null &&
                    pidController.getErrorDerivativeTolerance() != debugVelocityToleranceSupplier.getAsDouble()) {

                withDerivativeTolerance(debugPoseToleranceSupplier.getAsDouble());
            }
        }
    }
}
