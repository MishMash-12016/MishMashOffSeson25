package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class LinearIntake extends ServoSubsystem {
    public static double linerIntakeOpen = 0.5;
    public static double linerIntakeClose = 0.25;
    private static LinearIntake instance;


    public static synchronized LinearIntake getInstance() {
        if (instance == null) {
            instance = new LinearIntakeAutoLogged();
        }
        return instance;
    }
    public LinearIntake() {
        super("IntakeRotator");
        withServo("L linear intake ", Direction.FORWARD,0.0);
        withServo("R linear intake ", Direction.REVERSE,0.0);
    }
}
