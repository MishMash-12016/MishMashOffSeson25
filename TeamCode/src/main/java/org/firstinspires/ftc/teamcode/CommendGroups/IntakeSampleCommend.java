package org.firstinspires.ftc.teamcode.CommendGroups;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

import java.util.function.BooleanSupplier;

public class IntakeSampleCommend {
    public static Command prepareSampleIntake(BooleanSupplier rotateRightButton, BooleanSupplier rotateLeftButton) {
        return new ParallelCommandGroup(
                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowPrepareSampleTransferPose),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPrepareSampleTransferPose),
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawOpenPos),
                LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeOpen),
                IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmPrepareIntakeSample),
                IntakeRotator.getInstance().setPositionByButtonCommand(
                        IntakeRotator.defaultPose, rotateLeftButton, IntakeRotator.rotatorLeftAnglePose, rotateRightButton, IntakeRotator.rotatorRightAnglePose
                ),
                new WaitCommand(300).andThen(
                        IntakeClaw.getInstance().setPositionCommand(IntakeClaw.scoringClawOpen)
                )
        );
    }
}
