package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmDebug;

@TeleOp
public class ManualControlTest extends OpMode {
    private Arm arm;
    private ArmDebug armDebug;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);
    }

    @Override public void loop() {
        arm.update();

        double rotationInput = gamepad1.left_stick_y;
        double extensionInput = gamepad1.right_stick_y;

        if (Math.abs(rotationInput) > 0.0 && Math.abs(extensionInput) > 0.0) {
            arm.setManualMode();
            arm.manualControl(rotationInput, extensionInput);
        } else {
            arm.setPositionMode();

            if (gamepad1.dpad_left) {
                arm.setTargetInchesRobotCentric(5, 5);
            }
        }
    }
}
