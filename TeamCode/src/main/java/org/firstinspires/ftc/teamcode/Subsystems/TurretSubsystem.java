package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem.ZERO_SWITCH_PORT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.Position.PositionPidSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog

public class TurretSubsystem extends PositionPidSubsystem {

    public static double KP = 0.09;
    public static double KI = 0.001;
    public static double KD = 0.002;

    public static double KS = 0.045;
    public static double KG = 0.1;
    public static double KV = 0.047;
    public static double KA = 0.1;

    public static double CONSTRAINT_MAX_VELOCITY = 15;
    public static double CONSTRAINT_MAX_ACCELERATION = 100;
    public static double POSITION_TOLERANCE = 1;


    public static TurretSubsystemAutoLogged instance;



    public static synchronized TurretSubsystemAutoLogged getInstance() {
        if (instance == null) {
            instance = new TurretSubsystemAutoLogged("TurretSubsystem");
        }
        return instance;
    }

    public TurretSubsystem(String subsystemName) {
        super(subsystemName);

        MMRobot mmRobot = MMRobot.getInstance();

        withEncoder(mmRobot.controlHub,1,27.46, Direction.REVERSE);

        withMotor(mmRobot.controlHub, 1, Direction.REVERSE);
        withZeroSwitch(new CuttleDigital(mmRobot.expansionHub,0));
        withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PID & Constraints
        withPid(KP, KI, KD);

        // Tolerances
        withPositionTolerance(POSITION_TOLERANCE);

        // Zeroing limit switch on encoder
        withZeroSwitch(new CuttleDigital(mmRobot.expansionHub, 2),5 );

        // By default, hold whatever setpoint we’re at
        withSetDefaultCommand(holdCurrentSetPointCommand());


        withDebugPidSuppliers(
                ()-> KP,
                ()->KI,
                ()->KD,
                null,
                ()->POSITION_TOLERANCE,
                null,
                null,
                null
        );
    }

}
