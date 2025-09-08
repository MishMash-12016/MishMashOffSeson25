package org.firstinspires.ftc.teamcode.CommendGroups.Camera;

import com.acmerobotics.dashboard.config.Config;
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

import Ori.Coval.Logging.AutoLog;
import Ori.Coval.Logging.Logger.KoalaLog;

@Config
//@AutoLog
public class CameraCommandGroups {
    public static SequentialCommandGroup CameraSampleIntake(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> Camera.getInstance().switchToDetector()),
                new WaitUntilCommand(() -> Camera.getInstance().getPipelineIndex() == Camera.getInstance().currentPipeline),

                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowInitPose),
                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmInitPose),

                //Lamlam side:
                new InstantCommand(()->Camera.getInstance().setPreviousResult()),
                new InstantCommand(()-> KoalaLog.log("Previous result before angle change", Camera.getInstance().GetPreviousDetectorResult() != null,true)),
                Camera.getInstance().changeRotatorAngle(),
                new InstantCommand(()-> KoalaLog.log("Previous result after angle change", Camera.getInstance().GetPreviousDetectorResult() != null,true)),

//                new InstantCommand(() -> Camera.getInstance().switchToDetector()),
//                new WaitUntilCommand(() -> Camera.getInstance().getPipelineIndex() == Camera.getInstance().currentPipeline),
//                new InstantCommand(()->Camera.getInstance().setPreviousResult()),


                CameraCommands.StrafeToSample(),
//
                new WaitCommand(300),
                IntakeSampleCommend.prepareSampleIntakeNoIntakeRotator(),
                new WaitCommand(400),
                IntakeSampleCommend.SampleIntake()

                );
    }
}
