package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp
public class MecanumTest extends OpMode {
    private MecanumDrive mecanumDrive;

    @Override public void init() {
        mecanumDrive = new MecanumDrive.Builder(hardwareMap).build();
    }

    @Override public void loop() {
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        mecanumDrive.drive(
                leftStickY * Math.abs(leftStickY),
                leftStickX * Math.abs(leftStickX),
                rightStickX * Math.abs(rightStickX)
        );
    }
}
