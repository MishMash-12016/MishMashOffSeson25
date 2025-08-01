package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;

@TeleOp
public class TestNoMM extends CommandOpMode {
    Servo servo1;
    Servo servo2;
    @Override
    public void initialize() {
        servo1 = hardwareMap.servo.get("R outake arm");
        servo2 = hardwareMap.servo.get("L outake arm");
    }

    @Override
    public void run() {
        super.run();
        servo1.setPosition(gamepad1.left_stick_x);
        servo2.setPosition(gamepad1.left_stick_y);
    }
}
