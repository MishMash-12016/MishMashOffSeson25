package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.CommendGroups.IntakeSampleCommend;
import org.firstinspires.ftc.teamcode.CommendGroups.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommendGroups.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMDrivetrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeRotator;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntake;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringElbow;

import Ori.Coval.Logging.AutoLog;

@TeleOp
@Config
@AutoLog
public class TestOpMode extends MMOpMode {

    public TestOpMode() {
        super(OpModeType.NonCompetition.DEBUG);
    }

    boolean SpecimenIntake = false;

    @Override
    public void onInit() {

//        robotInstance.mmSystems.driveTrain.pose = MMSystems.AutoPose;
//        robotInstance.mmSystems.currentPose = MMSystems.AutoPose;

        //drive
        new Trigger(() -> MMRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whileActiveContinuous(
                MMDrivetrain.getInstance().driveCommand(
                        Math.pow(MMRobot.getInstance().gamepadEx1.getLeftX(), 5) * 0.3,
                        Math.pow(MMRobot.getInstance().gamepadEx1.getLeftY(), 5) * 0.3,
                        Math.pow(MMRobot.getInstance().gamepadEx1.getRightX(), 1) * 0.25
                )
        );

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> MMDrivetrain.getInstance().follower.resetOffset()
        );

        //prepareSampleIntake
        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                IntakeSampleCommend.prepareSampleIntake(
                        () -> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get(),
                        () -> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()
                ).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false)
                )
        );

        //prepareSpecimenIntake
        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                ScoringElbow.getInstance().setPositionCommand(ScoringElbow.ElbowPrepareSampleTransferPose),
                                IntakeRotator.getInstance().setPositionCommand(IntakeRotator.defaultPose),
                                ScoringArm.getInstance().setPositionCommand(ScoringArm.scoringArmPrepareSampleTransferPose),
                                LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeClose),
                                IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmSpecimenIntake),
                                IntakeClaw.getInstance().setPositionCommand(IntakeClaw.scoringClawOpen)

                        ),
                        IntakeSpecimenCommand.PrepareSpecimenIntakeFront().alongWith(
                                new InstantCommand(() -> SpecimenIntake = true)
                        ),
                        () -> (LinearIntake.getInstance().getPosition() == LinearIntake.linerIntakeOpen)
                )
        );

        //sample/specimen intake
        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(IntakeSpecimenCommand.SpecimenIntake(), IntakeSampleCommend.SampleIntake(), () -> SpecimenIntake)
        );

        new Trigger(() -> MMRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
                ScoringSampleCommand.PrepareHighSample()
        );
        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.ScoreSampleCommand()
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
