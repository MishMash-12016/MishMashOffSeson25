package org.firstinspires.ftc.teamcode.CommendGroups.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
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
//@AutoLog
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

    public static InstantCommand StrafeToSample() {
        LLResult lastResult = Camera.getInstance().GetPreviousDetectorResult();

        if (lastResult != null) {
            double distanceX = Camera.getInstance().getTx(Camera.getInstance().GetResult());
            double distanceY = Camera.getInstance().getTy(Camera.getInstance().GetResult());

            FtcDashboard.getInstance().getTelemetry().addData("distanceX in strafe", distanceX);
            FtcDashboard.getInstance().getTelemetry().addData("distanceY in strafe", distanceY);

            MMDrivetrain.getInstance().follower.updatePose();
            MMDrivetrain.getInstance().follower.update();

//            MMDrivetrain.update();
            Pose currentPose = MMDrivetrain.getInstance().follower.getPose();
            double h = currentPose.getHeading(); // radians

            // robot -> field
            double dxf = distanceX * Math.cos(h + Math.PI/2.0) + distanceY * Math.cos(h);
            double dyf = distanceX * Math.sin(h + Math.PI/2.0) + distanceY * Math.sin(h);

            // skip truly tiny nudges

            double endX = currentPose.getX() + dxf;
            double endY = currentPose.getY() + dyf;

            FtcDashboard.getInstance().getTelemetry().addData("endX in strafe", endX);
            FtcDashboard.getInstance().getTelemetry().addData("endY in strafe", endY);



            FtcDashboard.getInstance().getTelemetry().update();

            //TODO : implement pedro path to the endpoint, DONT KNOW IF IT IS RIGHT!
            Path strafeToSample = new Path(
                    // Line 1
                    new BezierLine(
                            new Point(MMDrivetrain.getInstance().follower.getPose().getX(),
                                    MMDrivetrain.getInstance().follower.getPose().getY(), Point.CARTESIAN),
                            new Point(endX, endY, Point.CARTESIAN))
            );
            double robotHeading = MMDrivetrain.getInstance().follower.getPose().getHeading(); // radians

            strafeToSample.setConstantHeadingInterpolation(robotHeading);
            new InstantCommand();
        }

        KoalaLog.log("last result is null", "", true);
        return new InstantCommand();
    }


}
