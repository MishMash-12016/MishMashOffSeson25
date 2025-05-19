package org.firstinspires.ftc.teamcode.learningPedero;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;

@Autonomous
public class PedroTest extends MMOpMode {

    private Follower follower;
    private PathBuilder builder;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose basketPose = new Pose(12, 133, Math.toRadians(-45));
    private final Pose yellowSampleClose = new Pose(40, 130, Math.toRadians(0));

    public PedroTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        follower = MMSystems.getInstance().initializeFollower(startPose);
        builder = follower.pathBuilder();
    }

    @Override
    public void onInitLoop() {}

    @Override
    public void onPlay() {
        schedule(
                new FollowPathCommand(follower,
                        builder.addPath(
                                new BezierCurve(
                                        new Point(startPose),
                                        new Point(12, 105),
                                        new Point(basketPose)
                                )
                        ).setLinearHeadingInterpolation(
                                startPose.getHeading(),
                                basketPose.getHeading()
                        ).build()
                ),
                new InstantCommand(),
                new FollowPathCommand(follower,
                        builder.addPath(
                                new BezierCurve(
                                        new Point(basketPose),
                                        new Point(25, 131),
                                        new Point(yellowSampleClose)
                                )
                        ).setLinearHeadingInterpolation(
                                basketPose.getHeading(),
                                yellowSampleClose.getHeading()
                        ).build()
                )
        );
    }

    @Override
    public void onPlayLoop() {
        follower.update();
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void onEnd() {}

}
