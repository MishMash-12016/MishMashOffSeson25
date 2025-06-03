package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Velocity.VelocityPidSubsystem;
import org.firstinspires.ftc.teamcode.MMSystems;

public class ShooterSubsystem extends VelocityPidSubsystem {
    public ShooterSubsystem(String subsystemName) {
        super(subsystemName);

        MMSystems mmSystems = MMSystems.getInstance();
//
//        withEncoder(mmSystems.controlHub, ENCODER_PORT, ENCODER_TICKS_PER_REV, Direction.REVERSE);
//
//        // Four drive motors, all reversed so that “forward” is upwards
//        withMotor(mmSystems.controlHub, RIGHT_SERVO_PORT, RIGHT_SERVO_DIRECTION);
//        withMotor(mmSystems.controlHub, LEFT_SERVO_PORT, LEFT_SERVO__DIRECTION);
//
//        // PIDF & Constraints
//        withPid(KP, KI, KD);
//        withIZone(I_ZONE);
//
//        // Tolerances
//        withPositionTolerance(POSITION_TOLERANCE);
//        withVelocityTolerance(VELOCITY_TOLERANCE);
//
//        // Zeroing limit switch on encoder
//        withZeroSwitch(new CuttleDigital(mmSystems.controlHub, ZERO_SWITCH_PORT), ZERO_POSE);
//
//        // By default, hold whatever setpoint we’re at
//        withSetDefaultCommand(holdCurrentSetPointCommand());
//
//        withDebugPidSuppliers(() -> KP, () -> KI, () -> KD, () -> I_ZONE, () -> POSITION_TOLERANCE, () -> VELOCITY_TOLERANCE,
//                null, null);
    }
}
