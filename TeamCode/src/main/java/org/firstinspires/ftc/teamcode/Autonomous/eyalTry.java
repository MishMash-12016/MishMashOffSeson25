package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInner;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.ArrayList;

import Ori.Coval.Logging.AutoLog;

@Autonomous
@Config
@AutoLog
public class eyalTry extends CommandOpMode {
    Follower follower = MMDrivetrain.getInstance().follower;
    public static double RADIUS = 10;
    PathChain fullPath = new PathChain(
            new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(RADIUS,0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN))),
            new Path(new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS,2*RADIUS, Point.CARTESIAN), new Point(0,2*RADIUS, Point.CARTESIAN))),
            new Path(new BezierCurve(new Point(0,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN))),
            new Path(new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)))
    );
//    Path path1 = new Path(
//            new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(RADIUS,0, Point.CARTESIAN), new Point(RADIUS, RADIUS, Point.CARTESIAN))
//    );
//    Path path2 = new Path(
//            new BezierCurve(new Point(RADIUS, RADIUS, Point.CARTESIAN), new Point(RADIUS,2*RADIUS, Point.CARTESIAN), new Point(0,2*RADIUS, Point.CARTESIAN))
//    );
//    Path path3 = new Path(
//            new BezierCurve(new Point(0,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS,2*RADIUS, Point.CARTESIAN), new Point(-RADIUS, RADIUS, Point.CARTESIAN))
//    );
//    Path path4 = new Path(
//            new BezierCurve(new Point(-RADIUS, RADIUS, Point.CARTESIAN), new Point(-RADIUS,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN))
//    );
    @Override
    public void initialize() {
        super.reset();


        schedule(
                // Updates follower to follow path
                new RunCommand(() -> follower.update()),

//                new FollowPathCommand(follower, path1),
//                new FollowPathCommand(follower, path2),
//                new FollowPathCommand(follower, path3),
//                new FollowPathCommand(follower, path4)

                new FollowPathCommand(follower, fullPath)
                );
    }

    @Override
    public void run() {
        super.run();

    }
}