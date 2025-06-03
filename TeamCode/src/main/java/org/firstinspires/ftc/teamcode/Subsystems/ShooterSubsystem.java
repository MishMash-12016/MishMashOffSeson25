package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Velocity.VelocityPidSubsystem;
import org.firstinspires.ftc.teamcode.MMSystems;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@AutoLogAndPostToFtcDashboard
public class ShooterSubsystem extends VelocityPidSubsystem {

    //–––––––––––––––––––––––––––––––––
    // Constants
    //–––––––––––––––––––––––––––––––––

    public static int ENCODER_PORT = 1;
    public static int ENCODER_TICKS_PER_REV = 400;

    public static int MOTOR_PORT = 0;


    public static Direction MOTOR_DIRECTION = Direction.FORWARD;

    public static double KP = 0.08;
    public static double KI = 0.001;
    public static double KD = 0.0;

    public static double KS = 0.1005;
    public static double KV = 0.1;
    public static double KA = 0.0;
    public static double I_ZONE = 0.5;

    public static double VELOCITY_TOLERANCE = 0.05;
    public static double ACCELERATION_TOLERANCE = Double.POSITIVE_INFINITY;

    public static int ZERO_SWITCH_PORT = 1;
    public static int ZERO_POSE = 0;
    public ShooterSubsystem(String subsystemName) {
        super(subsystemName);

        MMSystems mmSystems = MMSystems.getInstance();

        withEncoder(mmSystems.controlHub, ENCODER_PORT, ENCODER_TICKS_PER_REV, Direction.REVERSE);

        // Four drive motors, all reversed so that “forward” is upwards
        withMotor(mmSystems.controlHub, MOTOR_PORT, MOTOR_DIRECTION);

        // PIDF & Constraints
        withPid(KP, KI, KD);
        withFeedforward(KS, KV, KA);
        withIZone(I_ZONE);

        // Tolerances
        withVelocityTolerance(VELOCITY_TOLERANCE);
        withAccelerationTolerance(ACCELERATION_TOLERANCE);

        // By default, hold whatever setpoint we’re at
        withSetDefaultCommand(holdCurrentSetPointCommand());

        withDebugPidSuppliers(() -> KP, () -> KI, () -> KD, () -> I_ZONE, () -> VELOCITY_TOLERANCE, () -> ACCELERATION_TOLERANCE,
                null, null, ()-> KS, () -> KV, () -> KA);
    }
}
