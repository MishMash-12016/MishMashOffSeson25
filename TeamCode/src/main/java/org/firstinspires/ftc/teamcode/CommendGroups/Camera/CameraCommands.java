package org.firstinspires.ftc.teamcode.CommendGroups.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;

import java.util.Set;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.AutoLogOutput;
import Ori.Coval.Logging.Logger.KoalaLog;

@Config
@AutoLog
public class CameraCommands {

    public static double linearIntakeLength = 422; //TODO : CHECK IF TRUE!!!

    public static InstantCommand RotateToSampleCommand (){
        return new InstantCommand(()->{
            Double angle = Camera.getInstance().getSampleAngle();
            if (angle != null){
                if (((angle <= 15 || angle >= 165) && Camera.length < Camera.height)
                        || (angle >= 75 && angle <= 105 && Camera.length > Camera.height)) {
                    angle = 180 - angle;
                }
                if (angle >= 0 && angle <= 90) {
                    angle /= 270;
                    angle = IntakeRotator.defaultPose - angle;
                } else {
                    angle = 180 - angle;
                    angle /= 270;
                    angle = IntakeRotator.defaultPose + angle;
                }
                IntakeRotator.getInstance().setPosition(angle);
            }
        });
    }

    public static Command StrafeToSample() {

        LLResult lastResult = Camera.getInstance().GetPreviousDetectorResult();
        double distanceX = Camera.getInstance().getStrafeOffset(lastResult, 0, 0);
        double distanceY = (linearIntakeLength - Camera.getInstance().getDistance(lastResult, 0)) / 25.4;
        if (lastResult == null){
            KoalaLog.log("last result is null", "", true);
        }
        KoalaLog.log("distanceX in StrafeCommand ", distanceX, true);

        if (distanceX != 0) {
//            MMDrivetrain.update();
            Pose currentPose = MMDrivetrain.getInstance().follower.getPose();
            KoalaLog.log("entered the strafe if ", distanceX, true);
            Translation2d distanceXVector = new Translation2d(distanceX, currentPose.getHeading() + Math.toRadians(90));
            Translation2d distanceYVector = new Translation2d(distanceY, currentPose.getHeading());
            Translation2d endPoint = new Translation2d(currentPose.getX(), currentPose.getY())
                    .plus(distanceXVector)
                    .plus(distanceYVector);

            //TODO : implement pedro path to the endpoint, DONT KNOW IF IT IS RIGHT!
            Path strafeToSample = new Path(
                    // Line 1
                    new BezierLine(
                            new Point(MMDrivetrain.getInstance().follower.getPose().getX(),
                                    MMDrivetrain.getInstance().follower.getPose().getY(), Point.CARTESIAN),
                            new Point(endPoint.getX(), endPoint.getY(), Point.CARTESIAN))
            );

            return MMDrivetrain.getInstance().followPathCommand(strafeToSample);
        }
        return new InstantCommand();
    }


}
