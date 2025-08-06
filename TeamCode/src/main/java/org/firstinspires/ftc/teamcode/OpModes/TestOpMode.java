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
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
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

    public static Double power = 0.0;

    @Override
    public void onInit() {
        MMDrivetrain.getInstance().enableTeleopDriveDefaultCommand(() -> MMRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05);

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand(() -> MMDrivetrain.getInstance().resetYaw())
        );


        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                IntakeSampleCommend.prepareSampleIntake(
                        () -> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get(),
                        () -> MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()
                ).alongWith(
                        new InstantCommand(() -> SpecimenIntake = false)
                )
        );

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

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ConditionalCommand(IntakeSpecimenCommand.SpecimenIntake(), IntakeSampleCommend.SampleIntake(), () -> SpecimenIntake)
        );

        new Trigger(() -> MMRobot.getInstance().gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(
                ScoringSampleCommand.PrepareHighSample()
        );

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                ScoringSampleCommand.ScoreSampleCommand()
        );

//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmIntakeSample));
//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(IntakeArm.getInstance().setPositionCommand(IntakeArm.intakeArmTransferSample));
//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeOpen));
//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(LinearIntake.getInstance().setPositionCommand(LinearIntake.linerIntakeClose));

        ElevatorSubsystem.getInstance().setPose(0);

        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(ElevatorSubsystem.getInstance().getToSetpointCommand(10));
        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(ElevatorSubsystem.getInstance().getToSetpointCommand(0));

//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(IntakeClaw.getInstance().setPositionCommand(IntakeClaw.scoringClawClose));
//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(IntakeRotator.getInstance().setPositionCommand(IntakeRotator.rotatorRightAnglePose));
//        MMRobot.getInstance().gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(IntakeRotator.getInstance().setPositionCommand(IntakeRotator.rotatorLeftAnglePose));


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
