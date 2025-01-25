package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.MoveArmToPositionAction;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmDebug;

@Autonomous(group = "Test")
public final class ArmActionTest extends LinearOpMode {
    private Arm arm;
    private ArmDebug armDebug;

    @Override public void runOpMode() {
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);

        waitForStart();

        homeArm();

        runArmToPosition();

        while (true) {
            if (isStopRequested()) return;
            telemetry.addLine("Made It To Position!");
        }
    }

    private void homeArm() {
        while (!(arm.state() == Arm.ArmState.POSITION)) {
           arm.update();
        }
    }

    private void runArmToPosition() {
        arm.setTargetInchesRobotCentric(1.0, 11.5);

        ElapsedTime timer = new ElapsedTime();

        while (!arm.isAtPosition() && timer.seconds() < 4.0) {
            arm.update();
            armDebug.cartesian();
            armDebug.global();
            armDebug.polar();
            telemetry.update();
        }
    }
}
