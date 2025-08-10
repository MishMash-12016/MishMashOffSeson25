package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInner;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

import java.util.ArrayList;

import Ori.Coval.Logging.AutoLog;

@Autonomous
@Config
@AutoLog
public class eyalTry extends MMOpMode {
    Follower follower;
    public static double RADIUS = 4;
    PathChain fullPath = new PathChain(
            new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(0,10, Point.CARTESIAN)))
    );

    public eyalTry() {
        super(OpModeType.NonCompetition.DEBUG);
    }

    @Override
    public void onInit() {
        super.reset();
        follower = MMDrivetrain.getInstance().follower;

        //set default values to systems
        LinearIntake.getInstance().setPosition(LinearIntake.linerIntakeClose);
        IntakeArm.getInstance().setPosition(IntakeArm.intakeArmInit);

        addCommandsOnRun(
                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowInitPose),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmInitPose),
                MMDrivetrain.getInstance().followPathCommand(fullPath)

        );
    }

    @Override
    public void onInitLoop() {

    }

    @Override
    public void onPlay() {
    }

    @Override
    public void onPlayLoop() {

    }

    @Override
    public void onEnd() {

    }
}