package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;


@Config
@AutoLogAndPostToFtcDashboard
public class ScoringArm extends ServoSubsystem {

    public static double scoringArmInitPose = 0.52;
    public static double scoringArmSampleTransferPose = 0.54;
    public static double scoringArmPrepareSampleTransferPose = 0.45;
    public static double scoringArmScorePose = 0.19;
    public static double scoringArmIntakeFromFrontPose = 0.545;
    public static double scoringArmScoreFromFrontSpecimenPose = 0.42;
    public static double afterScoreFromFrontSpecimenPose = 0.3;
    public static double scoringArmScoreSample = 0.24;
    public static double scoringArmPark = 0.39;
    public static double estimatedPose = scoringArmInitPose;


    // Singleton instance
    private static ScoringArm instance;

    /**
     * Get the singleton instance of ElevatorSubsystem.
     */
    public static synchronized ScoringArm getInstance() {
        if (instance == null) {
            instance = new ScoringArmAutoLogged();
        }
        return instance;
    }

    /**
     * Creates a ServoSubsystem
     *
     */
    public ScoringArm() {
        super("ScoringArm");
        withServo("L outake arm ", Direction.FORWARD,0.015);
        withServo("R outake arm ", Direction.REVERSE,0.0);
    }
}
