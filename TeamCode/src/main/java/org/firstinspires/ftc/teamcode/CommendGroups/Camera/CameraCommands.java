package org.firstinspires.ftc.teamcode.CommendGroups.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.Logger.KoalaLog;

@Config
@AutoLog
public class CameraCommands {

    public static double linearIntakeLength = 422; //TODO : CHECK IF TRUE!!!
    public static double plusXdis = 20; //TODO : CHECK IF TRUE!!!
    public static double plusYdis = 20; //TODO : CHECK IF TRUE!!!

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

    public static CommandBase StrafeToSample() {
        return new CommandBase() {
            CommandBase strafeCommand = new InstantCommand();

            @Override
            public void initialize() {
                LLResult last = Camera.getInstance().GetPreviousDetectorResult();
                if (last == null) {
                    KoalaLog.log("last result is null", "", true);
                    return;
                }

                double dx = Camera.dx + plusXdis;
                double dy = Camera.dy + plusYdis;

                MMDrivetrain dt = MMDrivetrain.getInstance();
                dt.follower.updatePose();
                dt.follower.update();

                Pose p = dt.follower.getPose();
                double h = p.getHeading();

                double dxf = dx * Math.cos(h + Math.PI/2.0) + dy * Math.cos(h);
                double dyf = dx * Math.sin(h + Math.PI/2.0) + dy * Math.sin(h);

                double endX = p.getX() + dxf;
                double endY = p.getY() + dyf;

                Path path = new Path(new BezierLine(
                        new Point(p.getX(), p.getY(), Point.CARTESIAN),
                        new Point(endX, endY, Point.CARTESIAN)));
                path.setConstantHeadingInterpolation(h);

                // Schedule the actual followPath command now that we have live data
                strafeCommand = MMDrivetrain.getInstance().followPathCommand(path);
                strafeCommand.schedule();
            }

            @Override
            public boolean isFinished() {
                return strafeCommand.isFinished();
            }
        };
    }

}
