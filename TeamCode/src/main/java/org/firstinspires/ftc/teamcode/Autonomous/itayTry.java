package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;

@Autonomous

public class itayTry extends MMOpMode {


    public itayTry() {
        super(OpModeType.NonCompetition.DEBUG);
    }


    @Override
    public void onInit() {

        Follower follower = MMDrivetrain.getInstance().follower;

        MMDrivetrain.getInstance().setPose(7.178, 109.682, Math.toRadians(270));

        Path fromStartToScore = new Path(
                // Line 1
                new BezierCurve(
                        new Point(7.178, 109.682, Point.CARTESIAN),
                        new Point(13.458, 116.860, Point.CARTESIAN),
                        new Point(16.150, 126.280, Point.CARTESIAN)
                )
        );
        fromStartToScore.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(320));
        Path fromScoreToObservation = new Path(
                // Line 2
                new BezierCurve(
                        new Point(16.150, 126.280, Point.CARTESIAN),
                        new Point(64.150, 116.636, Point.CARTESIAN),
                        new Point(65.047, 97.346, Point.CARTESIAN)
                )
        );
        fromScoreToObservation.setTangentHeadingInterpolation();

        SequentialCommandGroup autonomous1 = new SequentialCommandGroup(
                MMDrivetrain.getInstance().followPathCommand(fromStartToScore),
                MMDrivetrain.getInstance().followPathCommand(fromScoreToObservation)
        );




        addCommandsOnRun(
                autonomous1
        );
    }

    @Override
    public void onPlayLoop() {

    }
}
