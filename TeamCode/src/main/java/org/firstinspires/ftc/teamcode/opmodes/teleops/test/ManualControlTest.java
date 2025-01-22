package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.*;

@TeleOp(group = "Test")
public final class ManualControlTest extends OpMode {
    private Arm arm;
    private ArmDebug armDebug;
    private boolean initializedManualMode, initializedPositionMode;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);
        initializedManualMode = false;
        initializedPositionMode = false;
    }

    @Override public void loop() {
        arm.update();

        double rotationInput = -gamepad1.left_stick_y;
        double extensionInput = -gamepad1.right_stick_y;

        double rotationInputMagnitude = Math.abs(rotationInput);
        double extensionInputMagnitude = Math.abs(extensionInput);

        if (rotationInputMagnitude <= 0.1) rotationInput = 0.0;
        if (extensionInputMagnitude <= 0.1) extensionInput = 0.0;

        telemetry.addData("Rotation Input", rotationInput);
        telemetry.addData("Extension Input", extensionInput);

        if (rotationInputMagnitude > 0.0 || extensionInputMagnitude > 0.0) {
            telemetry.addLine("Manual If Statement");
            arm.setManualMode();

            arm.manualControl(rotationInput, extensionInput);
        } else {
            telemetry.addLine("Position If Statement");
            arm.setPositionMode();

            if (gamepad1.dpad_left) arm.setTargetInchesRobotCentric(5.0, 5.0);
        }

        armDebug.global();
        armDebug.polar();
        armDebug.cartesian();
    }
}