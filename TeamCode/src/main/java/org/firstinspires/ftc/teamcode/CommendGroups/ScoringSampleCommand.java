package org.firstinspires.ftc.teamcode.CommendGroups;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

public class ScoringSampleCommand {

    public static Command ScoreSampleCommand(){
        return new SequentialCommandGroup(
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawOpenPos),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        ElevatorSubsystem.getInstance().getToAndHoldSetPointCommand(0),
                        IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmSpecimenIntake),
                        ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPrepareSampleTransferPose),
                        ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowPrepareSampleTransferPose)
                )
        );
    }

    public static Command PrepareHighSample(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        ElevatorSubsystem.getInstance().getToAndHoldSetPointCommand(ElevatorSubsystem.ELEVATOR_HIGH_BASKET),
                        IntakeRotator.getInstance().setPositionCommand(IntakeRotator.initPose),
                        LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeClose),
                        IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmTransferSample),
                        ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowTransferSamplePose),
                        ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPrepareSampleTransferPose)
                ),
                new WaitCommand(50),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmSampleTransferPose),
                new WaitCommand(50),
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawClosePos),
                new WaitCommand(100),
                IntakeClaw.getInstance().setPositionCommand(IntakeClaw.scoringClawOpen),
                new WaitCommand(100),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmScorePose),
                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.prepareSampleScorePose),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        ElevatorSubsystem.getInstance().getToAndHoldSetPointCommand(ElevatorSubsystem.ELEVATOR_HIGH_BASKET),
                        new WaitCommand(200).andThen(
                                IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmPrepareIntakeSample)
                        ),
                        new WaitUntilCommand(() -> ElevatorSubsystem.getInstance().getPose() > 37).andThen(
                                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.prepareSampleScorePose)
                        )
                ),
                new InstantCommand(() -> ElevatorSubsystem.getInstance().setPower(0), ElevatorSubsystem.getInstance())
        );
    }
}
