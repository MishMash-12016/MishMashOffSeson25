package org.firstinspires.ftc.teamcode.CommendGroups.Camera;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

import Ori.Coval.Logging.Logger.KoalaLog;


@Config
public class strafeToSample extends CommandBase {
    public static double maxDistanceY = 422;
    public static double maxDistanceYShort = 422;
    public static double accelerationMultiplierShort = 0.72;
    public static double limit = 10;
    public static double theOtherSide = -2.5;
    public static double theOtherSideAdder = 2.2;
    public static double accelerationMultiplierLong = 1;
    Boolean finished = true;

    boolean isSample;
    public static double samplePlusDis = 0.1;
    public static double plusDistanceX = 0.7;


    Boolean found = false;
    public strafeToSample() {
        addRequirements(
                MMDrivetrain.getInstance()
        );
    }


    @Override
    public void initialize() {
        LLResult lastResult = Camera.getInstance().GetPreviousDetectorResult();
        double distanceX = Camera.getInstance().getStrafeOffset(lastResult);
        double distanceY = (maxDistanceY - Camera.getInstance().getDistance(lastResult)) / 25.4;

        KoalaLog.log("distanceX - ", distanceX, true);
        KoalaLog.log("distanceY - ", distanceY, true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}