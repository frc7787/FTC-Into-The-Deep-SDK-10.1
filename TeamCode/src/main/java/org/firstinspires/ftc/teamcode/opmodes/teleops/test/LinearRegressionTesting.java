package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmDebug;

@TeleOp
public class LinearRegressionTesting extends OpMode {
    private Arm arm;
    private ArmDebug armDebug;

    @Override public void init() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);
    }

    @Override public void loop() {
       arm.update();
       arm.manualControlSub(-gamepad2.left_stick_y, 2.0);
       armDebug.cartesianPosition();
    }
}
