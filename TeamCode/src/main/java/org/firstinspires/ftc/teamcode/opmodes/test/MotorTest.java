package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ArmSubsystem;

@TeleOp(name = "Test - Arm Subsystem", group = "Test")
public final class MotorTest extends OpMode {
    private ArmSubsystem armSubsystem;

    @Override public void init() {
        armSubsystem = new ArmSubsystem(hardwareMap);
    }

    @Override public void loop() {
        armSubsystem.setExtensionPower(gamepad1.left_stick_y * -1.0);
        armSubsystem.setRotationPower(gamepad1.right_stick_y * -1.0);
    }
}
