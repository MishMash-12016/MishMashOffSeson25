package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position.PositionPidSubsystem;
import org.firstinspires.ftc.teamcode.MMSystems;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@AutoLogAndPostToFtcDashboard
public class TurretSubsystem extends PositionPidSubsystem {

    //–––––––––––––––––––––––––––––––––
    // Constants
    //–––––––––––––––––––––––––––––––––

    public static int ENCODER_PORT = 1;
    public static int ENCODER_TICKS_PER_REV = 400;

    public static int RIGHT_SERVO_PORT = 0;
    public static int LEFT_SERVO_PORT = 1;


    public static Direction RIGHT_SERVO_DIRECTION = Direction.FORWARD;
    public static Direction LEFT_SERVO__DIRECTION = Direction.REVERSE;

    public static double KP = 0.08;
    public static double KI = 0.001;
    public static double KD = 0.0;
    public static double I_ZONE = 0.5;

    public static double POSITION_TOLERANCE = 0.05;
    public static double VELOCITY_TOLERANCE = Double.POSITIVE_INFINITY;

    public static int ZERO_SWITCH_PORT = 1;
    public static int ZERO_POSE = 0;

    public TurretSubsystem(String subsystemName) {
        super(subsystemName);

        MMSystems mmSystems = MMSystems.getInstance();

        withEncoder(mmSystems.controlHub, ENCODER_PORT, ENCODER_TICKS_PER_REV, Direction.REVERSE);

        // Four drive motors, all reversed so that “forward” is upwards
        withCrServo(mmSystems.controlHub, RIGHT_SERVO_PORT, RIGHT_SERVO_DIRECTION);
        withCrServo(mmSystems.controlHub, LEFT_SERVO_PORT, LEFT_SERVO__DIRECTION);

        // PIDF & Constraints
        withPid(KP, KI, KD);
        withIZone(I_ZONE);

        // Tolerances
        withPositionTolerance(POSITION_TOLERANCE);
        withVelocityTolerance(VELOCITY_TOLERANCE);

        // Zeroing limit switch on encoder
        withZeroSwitch(new CuttleDigital(mmSystems.controlHub, ZERO_SWITCH_PORT), ZERO_POSE);

        // By default, hold whatever setpoint we’re at
        withSetDefaultCommand(holdCurrentSetPointCommand());

        withDebugPidSuppliers(() -> KP, () -> KI, () -> KD, () -> I_ZONE, () -> POSITION_TOLERANCE, () -> VELOCITY_TOLERANCE,
                null, null);
    }
}
