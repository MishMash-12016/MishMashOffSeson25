package org.firstinspires.ftc.teamcode.CommendGroups.Camera;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.CommendGroups.IntakeSampleCommend;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

public class CameraIntake {
    public static SequentialCommandGroup CameraSampleIntake(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> Camera.getInstance().switchToDetector()),
                new WaitUntilCommand(() -> Camera.getInstance().getPipelineIndex() == Camera.getInstance().currentPipeline),

                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowPrepareSampleTransferPose),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPrepareSampleTransferPose),
                ScoringClaw.getInstance().setPositionCommand(ScoringClaw.IntakeClawOpenPos),
                IntakeArm.getInstance().setPositionOverTimeCommand(IntakeArm.intakeArmPrepareIntakeSample, 300),
                new WaitCommand(300).andThen(
                        IntakeClaw.getInstance().setPositionCommand(IntakeClaw.scoringClawOpen)
                ),

                //Lamlam side:
                new InstantCommand(() -> Camera.getInstance().setPreviousResult()),
                Camera.getInstance().changeRotatorAngle(),

                CameraCommands.StrafeToSample(),

                LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeOpen),
                new WaitCommand(400),
                IntakeSampleCommend.SampleIntake()

                );
    }
}
