package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.OpModeVeriables.OpModeType;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LinearIntakeSubsystem;

@TeleOp
public class ExampleTeleopOpMode extends MMOpMode {
    MMSystems mmSystems;

    public ExampleTeleopOpMode() {
        super(OpModeType.Competition.TELEOP);
    }

    @Override
    public void onInit() {
        mmSystems = MMSystems.getInstance();
        GamepadEx gamepadEx1 = mmSystems.gamepadEx1;
        GamepadEx gamepadEx2 = mmSystems.gamepadEx2;

        //prepare score HIGH_BASKET
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenActive(ElevatorSubsystem.getInstance().moveToStateCommand(ElevatorSubsystem.ElevatorState.HIGH_BASKET));

        //prepare intake
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                                ElevatorSubsystem.getInstance().moveToStateCommand(ElevatorSubsystem.ElevatorState.ELEVATOR_DOWN),
                                IntakeArmSubsystem.getInstance().setStateCommand(IntakeArmSubsystem.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                LinearIntakeSubsystem.getInstance().openCommand()));

        //using second gamepad to manually reset elevator pose
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()->ElevatorSubsystem.getInstance().setPose(ElevatorSubsystem.ElevatorState.ELEVATOR_ZERO_POSE.getPosition()));

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
