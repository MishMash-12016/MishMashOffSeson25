package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class ScoringClaw extends ServoSubsystem {
    public static double scoringClawOpen = 0.48;
    public static double IntakeClawOpenPos = 0.95;
    private static ScoringClaw instance;
    public static synchronized ScoringClaw getInstance(){
        if (instance == null) {
            instance = new ScoringClawAutoLogged();
        }
        return instance;
    }

    public ScoringClaw() {
        super("ScoringClaw");
        withServo("intake claw", Direction.FORWARD,0.0);
    }



}
