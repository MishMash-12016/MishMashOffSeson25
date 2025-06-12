package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;

import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.Libraries.pedroPathing.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.MMRobot;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class MMDrivetrain extends SubsystemBase {
    public Follower follower;

    private static MMDrivetrain instance;

    public static synchronized MMDrivetrain getInstance() {
        if (instance == null) {
            instance = new MMDrivetrainAutoLogged();
        }
        return instance;
    }

    public MMDrivetrain() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(MMRobot.getInstance().currentOpMode.hardwareMap);
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
    public void enableDefaultCommand() {
        MMRobot mmRobot = MMRobot.getInstance();
        setDefaultCommand(driveCommand(-mmRobot.gamepadEx1.getLeftY(), -mmRobot.gamepadEx1.getLeftX(), -mmRobot.gamepadEx1.getRightX()));
    }

    /**
     * disables the Default Command(the default command is the drive field centric command)
     */
    public void disableDefaultCommand() {
        setDefaultCommand(new RunCommand(()->{}, this));
    }
}
