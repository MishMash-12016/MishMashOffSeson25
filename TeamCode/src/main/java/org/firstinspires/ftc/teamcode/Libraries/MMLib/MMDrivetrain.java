package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;

public class MMDrivetrain extends SubsystemBase {
    public Follower follower;

    public static MMDrivetrain instance;

    public static synchronized MMDrivetrain getInstance() {
        if (instance == null) {
            instance = new MMDrivetrain();
        }
        return instance;
    }

    public static void resetFollower(){
        if(instance!=null){
            instance.follower.initialize();
        }
    }

    public static void update(){
        if(instance != null){
            try {
                instance.follower.update();             //updates the follower
                instance.follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());//puts pedro data(robot pose, speed..) on the FtcDashboard
            } catch (Exception e) {
                //todo: fix weird pedro problem
            }
        }
    }

    public MMDrivetrain() {
        follower = new Follower(MMRobot.getInstance().currentOpMode.hardwareMap, FConstants.class, LConstants.class);
    }

    public CommandBase holdPointCommand(Pose pose) {
        CommandBase holdPointCommand = new HoldPointCommand(follower, pose, false);
        holdPointCommand.addRequirements(this);

        return holdPointCommand;
    }

    public CommandBase followPathCommand(Path path, boolean holdEnd) {
        CommandBase followPathCommand = new FollowPathCommand(follower, path, holdEnd);
        followPathCommand.addRequirements(this);

        return followPathCommand;
    }

    public CommandBase followPathCommand(Path path) {
        return this.followPathCommand(path, FollowerConstants.automaticHoldEnd);
    }

    public CommandBase followPathCommand(PathChain pathChain) {
        CommandBase followPathCommand = new FollowPathCommand(follower, pathChain, FollowerConstants.automaticHoldEnd);
        followPathCommand.addRequirements(this);

        return followPathCommand;
    }

    public CommandBase driveCommand(double forwardDrive, double lateralDrive, double heading) {
        return this.driveCommand(forwardDrive, lateralDrive, heading, true);
    }

    public CommandBase driveCommand(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        return (CommandBase) new RunCommand(() -> {
            follower.setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, robotCentric);
            follower.update();
        }, this)
                .beforeStarting(() -> follower.startTeleopDrive());
    }

    public CommandBase turnCommand(double radians, boolean isLeft) {
        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() + (isLeft ? radians : -radians));
        return this.holdPointCommand(temp);
    }

    public CommandBase turnToCommand(double radians) {
        return this.holdPointCommand(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(radians)));
    }

    public CommandBase turnToDegreesCommand(double degrees) {
        return this.turnToCommand(Math.toRadians(degrees));
    }

    public CommandBase turnDegreesCommand(double degrees, boolean isLeft) {
        return this.turnCommand(Math.toRadians(degrees), isLeft);
    }

    /**
     * enables the Default Command(the default command is the drive field centric command)
     */
    public void enableTeleopDriveDefaultCommand() {
        MMRobot mmRobot = MMRobot.getInstance();
        setDefaultCommand(driveCommand(-mmRobot.gamepadEx1.getLeftY(), -mmRobot.gamepadEx1.getLeftX(), -mmRobot.gamepadEx1.getRightX()));
    }

    /**
     * disables the Default Command(the default command is the drive field centric command)
     */
    public void disableTeleopDriveDefaultCommand() {
        setDefaultCommand(new RunCommand(()->{}, this));
    }
}
