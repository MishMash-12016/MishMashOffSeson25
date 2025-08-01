package org.firstinspires.ftc.teamcode.CommendGroups;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

public class IntakeSpecimenCommand {
    public static Command SpecimenIntake() {
        return new SequentialCommandGroup(
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawClosePos),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        ScoringArm.getInstance().setPositionOverTimeCommand(ScoringArm.scoringArmScorePose,300),
                        new SequentialCommandGroup(
                                new WaitCommand(50),
                                ScoringElbow.getInstance().setPositionCommand(0.35),//TODO:whad dis iz?
                                new WaitCommand(50),
                                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowScoreSpecimenPose)
                        )//TODO:whad dis iz?
                )
        );
    }

    public static Command PrepareSpecimenIntakeFront() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeClose),
                        IntakeRotator.getInstance().setPositionCommand(IntakeRotator.initPose),
                        IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmInit)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowIntakeFromFrontPose),
                        ScoringArm.getInstance().setPositionOverTimeCommand(ScoringArm.scoringArmIntakeFromFrontPose, 400),
                        ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawOpenPos)
                )
        );
    }
}
