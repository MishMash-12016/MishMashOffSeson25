package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import java.util.function.Supplier;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class ScoringElbow extends ServoSubsystem  {
    public static double prepareSampleScorePose = 0.42;
    public static double ElbowScoreSamplePose = 0.78;
    public static double ElbowInitPose = 0.32;
    public static double ElbowTransferSamplePose = 0.1;
    public static double ElbowPrepareSampleTransferPose = 0.05;
    public static double ElbowScoreSpecimenPose = 0.74;
    public static double ElbowAfterScoreSpecimenPose = 0.64;
    public static double ElbowIntakeFromFrontPose = 0.4;
    public static double scoringElbowScoreFromFrontPose = 0.53;
    public static double ElbowAfterScoreFromFront = 0.35;
    public static double ElbowPark = 0.38;
    public static double est = 0;

    private static ScoringElbow instance;

    public static synchronized ScoringElbow getInstance() {
        if (instance == null) {
            instance = new ScoringElbowAutoLogged();
        }
        return instance;
    }
    public ScoringElbow() {
        super("ScoringElbow");
        withServo("ScoringElbowServo", Direction.FORWARD, 0.0);
    }

}
