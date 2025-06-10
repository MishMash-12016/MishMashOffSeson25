package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position.PositionProfiledPidSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.Supplier;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

/**
 * ElevatorSubsystem
 * <p>
 * A profiled‐PID subsystem that controls the robot’s elevator via four motors
 * and a zeroing limit switch on an encoder.
 */
@SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
@Config
@AutoLogAndPostToFtcDashboard
public class ElevatorSubsystem extends PositionProfiledPidSubsystem {

    //–––––––––––––––––––––––––––––––––
    // Constants
    //–––––––––––––––––––––––––––––––––

    public static int ENCODER_PORT = 1;
    public static int ENCODER_TICKS_PER_REV = 400;

    public static int MOTOR_PORT_0 = 0;
    public static int MOTOR_PORT_1 = 1;
    public static int MOTOR_PORT_2 = 2;
    public static int MOTOR_PORT_3 = 3;

    public static Direction MOTOR_DIRECTION_0 = Direction.FORWARD;
    public static Direction MOTOR_DIRECTION_1 = Direction.FORWARD;
    public static Direction MOTOR_DIRECTION_2 = Direction.REVERSE;
    public static Direction MOTOR_DIRECTION_3 = Direction.REVERSE;


    public static DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

    public static double KP = 0.08;
    public static double KI = 0.001;
    public static double KD = 0.0;

    public static double KS = 0.1005;
    public static double KV = 0.1;
    public static double KA = 0.0;

    public static double CONSTRAINT_MAX_VELOCITY = 0.5;
    public static double CONSTRAINT_MAX_ACCELERATION = 5.0;

    public static double I_ZONE = 0.5;
    public static double POSITION_TOLERANCE = 0.05;
    public static double VELOCITY_TOLERANCE = 0.0;

    public static int ZERO_SWITCH_PORT = 0;
    public static int ZERO_POSE = 0;

    // Predefined elevator setpoint positions (in inches)
    public static double ELEVATOR_HIGH_BASKET = 48.0;
    public static double ELEVATOR_DOWN_POSITION = 0.0;
    public static double ELEVATOR_CLIMB_HIGH_BAR = 64.0;
    public static double ELEVATOR_ZERO = 64.0;
    public static double ELEVATOR_CLIMB_POSITION = 40.0;

    //–––––––––––––––––––––––––––––––––
    // Enum: Named setpoints
    //–––––––––––––––––––––––––––––––––

    public enum ElevatorState {
        HIGH_BASKET(() -> ELEVATOR_HIGH_BASKET),
        ELEVATOR_DOWN(() -> ELEVATOR_DOWN_POSITION),
        ELEVATOR_HIGH_CHAMBER(() -> ELEVATOR_CLIMB_HIGH_BAR),
        ELEVATOR_ZERO_POSE(() -> ELEVATOR_ZERO),
        ELEVATOR_CLIMB(() -> ELEVATOR_CLIMB_POSITION);

        public final Supplier<Double> positionSupplier;

        ElevatorState(Supplier<Double> positionSupplier) {
            this.positionSupplier = positionSupplier;
        }

        public double getPosition() {
            return positionSupplier.get();
        }
    }

    //–––––––––––––––––––––––––––––––––
    // Instance fields
    //–––––––––––––––––––––––––––––––––


    // Singleton instance
    public static ElevatorSubsystemAutoLogged instance;

    /**
     * Get the singleton instance of ElevatorSubsystem.
     */
    public static synchronized ElevatorSubsystemAutoLogged getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystemAutoLogged("ElevatorSubsystem");
        }
        return instance;
    }


    /**
     * public constructor; use getInstance() for singleton access.
     */
    public ElevatorSubsystem(String subsystemName) {
        super(subsystemName);

        MMRobot mmRobot = MMRobot.getInstance();

        withEncoder(mmRobot.controlHub, ENCODER_PORT, ENCODER_TICKS_PER_REV, Direction.REVERSE);

        // Four drive motors, all reversed so that “forward” is upwards
        withMotor(mmRobot.controlHub, MOTOR_PORT_0, MOTOR_DIRECTION_0);
        withMotor(mmRobot.controlHub, MOTOR_PORT_1, MOTOR_DIRECTION_1);
        withMotor(mmRobot.controlHub, MOTOR_PORT_2, MOTOR_DIRECTION_2);
        withMotor(mmRobot.controlHub, MOTOR_PORT_3, MOTOR_DIRECTION_3);

        withZeroPowerBehavior(zeroPowerBehavior);

        // PIDF & Constraints
        withPid(KP, KI, KD);
        withFeedforward(KS, KV, KA);
        withConstraints(CONSTRAINT_MAX_VELOCITY, CONSTRAINT_MAX_ACCELERATION);
        withIZone(I_ZONE);

        // Tolerances
        withPositionTolerance(POSITION_TOLERANCE);
        withVelocityTolerance(VELOCITY_TOLERANCE);

        // Zeroing limit switch on encoder
        withZeroSwitch(new CuttleDigital(mmRobot.controlHub, ZERO_SWITCH_PORT), ZERO_POSE);

        // By default, hold whatever setpoint we’re at
        withSetDefaultCommand(holdCurrentSetPointCommand());

        withDebugPidSuppliers(() -> KP, () -> KI, () -> KD, () -> I_ZONE, () -> POSITION_TOLERANCE, () -> VELOCITY_TOLERANCE,
                null, null, () -> KS, () -> KV, () -> KA,
                () -> CONSTRAINT_MAX_VELOCITY, () -> CONSTRAINT_MAX_ACCELERATION);
    }

    //–––––––––––––––––––––––––––––––––
    // Public Commands
    //–––––––––––––––––––––––––––––––––

    /**
     * Returns a command that drives the elevator to one of the named states.
     * <p>
     * Example:
     * elevator.moveToState(ElevatorState.HIGH_BASKET);
     */
    public Command moveToStateCommand(ElevatorState state) {
        return getToSetpointCommand(state.getPosition());
    }
}
