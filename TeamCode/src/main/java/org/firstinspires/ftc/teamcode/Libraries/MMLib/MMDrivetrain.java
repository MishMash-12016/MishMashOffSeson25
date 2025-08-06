package org.firstinspires.ftc.teamcode.Libraries.MMLib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
        if(instance != null && instance.follower!=null){
            instance.follower.update();             //updates the follower
//            instance.follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());//puts pedro data(robot pose, speed..) on the FtcDashboard//TODO: i dont know what but this needs to be fixed
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

    public CommandBase driveCommand(DoubleSupplier forwardDrive, DoubleSupplier lateralDrive, DoubleSupplier heading, BooleanSupplier slowMode) {
        return this.driveCommand(forwardDrive, lateralDrive, heading, false, slowMode);
    }

    public CommandBase driveCommand(DoubleSupplier forwardDrive, DoubleSupplier lateralDrive, DoubleSupplier heading, boolean robotCentric, BooleanSupplier slowMode) {
        return (CommandBase) new RunCommand(() -> {

            if (slowMode.getAsBoolean()) {
                follower.setTeleOpMovementVectors(
                        Math.pow(forwardDrive.getAsDouble(), 5) * 0.3,
                        Math.pow(lateralDrive.getAsDouble(), 5) * 0.3,
                        Math.pow(heading.getAsDouble(), 1) * 0.25,
                        robotCentric);
            }
            else {
                follower.setTeleOpMovementVectors(forwardDrive.getAsDouble(), lateralDrive.getAsDouble(), heading.getAsDouble(), robotCentric);
            }
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

    public void resetYaw(){
        Pose pose = follower.getPose();
        pose.setHeading(0);
        follower.setPose(pose);
    }

    /**
     * enables the Default Command(the default command is the drive field centric command)
     */
    public void enableTeleopDriveDefaultCommand(BooleanSupplier slowMode) {
        MMRobot mmRobot = MMRobot.getInstance();
        setDefaultCommand(driveCommand(
                ()-> mmRobot.gamepadEx1.getLeftY(),
                ()-> -mmRobot.gamepadEx1.getLeftX(),
                ()-> -mmRobot.gamepadEx1.getRightX(),
                false, slowMode)
        );
    }

    /**
     * disables the Default Command(the default command is the drive field centric command)
     */
    public void disableTeleopDriveDefaultCommand() {
        setDefaultCommand(new RunCommand(()->{}, this));
    }
}
