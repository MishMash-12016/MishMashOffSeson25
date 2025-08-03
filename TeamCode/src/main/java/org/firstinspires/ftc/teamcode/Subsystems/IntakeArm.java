package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLog;

@Config
@AutoLog
public class IntakeArm extends ServoSubsystem {
    public static double intakeArmIntakeSample = 0.65;
    public static double intakeArmPrepareIntakeSample = 0.58;
    public static double intakeArmSpecimenIntake = 0.28;
    public static double intakeArmTransferSample = 0.11;
    public static double intakeArmInit = 0.13;
    private static IntakeArm instance;


    public static synchronized IntakeArm getInstance() {
        if (instance == null) {
            instance = new IntakeArmAutoLogged();
        }
        return instance;
    }
    public IntakeArm() {
        super("IntakeArm");
        withServo("L intake arm", Direction.FORWARD,0.0);
        withServo("R intake arm", Direction.REVERSE,0.015);
    }
}
