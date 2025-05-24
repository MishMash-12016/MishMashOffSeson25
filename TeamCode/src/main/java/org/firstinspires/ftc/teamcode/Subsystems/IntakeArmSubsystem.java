package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.function.Supplier;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class IntakeArmSubsystem extends ServoSubsystem {
    public static double intakeArmIntakeSamplePos = 0.57;
    public static double intakeArmPrepareIntakeSamplePose = 0.5;
    public static double intakeArmSpecimenIntakePose = 0.2;
    public static double intakeArmTransferSamplePose = 0.03;
    public static double intakeArmInitPose = 0.05;

    public enum IntakeArmState {
        SAMPLE_INTAKE_POSE(()-> intakeArmIntakeSamplePos),
        PREPARE_SAMPLE_INTAKE(()-> intakeArmPrepareIntakeSamplePose),
        SPECIMEN_INTAKE(()-> intakeArmSpecimenIntakePose),
        SAMPLE_TRANSFER_POSE(()-> intakeArmTransferSamplePose),
        INIT_POSE(()-> intakeArmInitPose);

        public Supplier<Double> position;

        IntakeArmState(Supplier<Double> position) {
            this.position = position;
        }
    }

    MMSystems mmSystems = MMSystems.getInstance();
    public IntakeArmSubsystem(String subsystemName) {
        super(subsystemName);
        withServo(mmSystems.controlHub, 5, Direction.REVERSE, 0.0);
        withServo(mmSystems.controlHub, 6, Direction.FORWARD, 0.0);
    }

    public Command setIntakeArmState(IntakeArmState state) {
        return setPositionCommand(state.position.get());
    }
}
