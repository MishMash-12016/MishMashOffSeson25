package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.utils.Direction;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Servo.ServoSubsystem;
import org.firstinspires.ftc.teamcode.MMSystems;

import java.util.function.Supplier;

import Ori.Coval.Logging.AutoLogAndPostToFtcDashboard;

@Config
@AutoLogAndPostToFtcDashboard
public class IntakeArmSubsystem extends ServoSubsystem {

    public static int SERVO_PORT_1 = 5;
    public static int SERVO_PORT_2 = 6;
    public static Direction SERVO_DIRECTION_1 = Direction.FORWARD;
    public static Direction SERVO_DIRECTION_2 = Direction.REVERSE;
    public static double SERVO_OFFSET_1 = 0.0;
    public static double SERVO_OFFSET_2 = 0.015;

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

    // Singleton instance
    public static IntakeArmSubsystemAutoLogged instance;

    /**
     * Get the singleton instance of IntakeArmSubsystem.
     */
    public static synchronized IntakeArmSubsystemAutoLogged getInstance() {
        if (instance == null) {
            instance = new IntakeArmSubsystemAutoLogged("IntakeArmSubsystem");
        }
        return instance;
    }

    public IntakeArmSubsystem(String subsystemName) {
        super(subsystemName);

        MMSystems mmSystems = MMSystems.getInstance();
        withServo(mmSystems.controlHub, SERVO_PORT_1, SERVO_DIRECTION_1, SERVO_OFFSET_1);
        withServo(mmSystems.controlHub, SERVO_PORT_2, SERVO_DIRECTION_2, SERVO_OFFSET_2);
    }

    public Command setStateCommand(IntakeArmState state) {
        return setPositionCommand(state.position.get());
    }
}
