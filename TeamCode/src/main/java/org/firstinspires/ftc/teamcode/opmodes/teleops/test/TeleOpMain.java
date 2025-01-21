package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.DriveMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmDebug;

@TeleOp(group="$")
public class TeleOpMain extends OpMode {
    private MecanumDrive drive;
    private Arm arm;
    private ArmDebug armDebug;

    private final double INTAKE_OPEN_POSITION = 0.50;
    private final double INTAKE_SUB_PRIMED_POSITION = 0.35;
    private final double INTAKE_CLOSED_POSITION = 0.00;
    private final double INTAKE_SPECIMEN_PICKUP_POSITION = 0.25;
    private final double INTAKE_CLIPPING_POSITION = 0.22;

    private final double BUCKET_VERTICAL_POSITION = 40.0;
    private final double BUCKET_HORIZONTAL_POSITION = 5;
    private final double BAR_VERTICAL_POSITION = 24.5;
    private final double BAR_HORIZONTAL_POSITION = 1.5;

    private final double SUB_VERTICAL_POSITION = 0.4;
    private final double SUB_HORIZONTAL_POSITION = 2.0;

    private final double NEUTRAL_VERTICAL_POSITION = 6.0;
    private final double NEUTRAL_HORIZONTAL_POSITION = 3.0;

    private final double GROUND_VERTICAL_POSITION = -2.0;
    private final double GROUND_HORIZONTAL_POSITION = 0.0;

    private final double HOVER_VERTICAL_POSITION = 0.3;

    private final double WALL_VERTICAL_INCHES = 11;
    private final double WALL_HORIZONTAL_INCHES = 2;

    private Gamepad previousGamepad2, currentGamepad2, previousGamepad1, currentGamepad1;

    private ArmState armState;

    @Override public void init() {
        drive = new MecanumDrive.Builder(hardwareMap)
                .setDriveMode(DriveMode.ROBOT_CENTRIC)
                .build();
        arm = new Arm(hardwareMap);
        armDebug = new ArmDebug(arm, telemetry);
        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        armState = ArmState.HOMING;
    }

    @Override public void loop() {
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        drive.drive(
                leftStickY * Math.abs(leftStickY),
                leftStickX * Math.abs(leftStickX),
                rightStickX * Math.abs(rightStickX)
        );

        if (gamepad2.left_bumper) {
            arm.setIntakePosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(INTAKE_CLOSED_POSITION);
        }

        switch (armState) {
            case HOMING:
                if (arm.state() != Arm.ArmState.HOMING) { armState = ArmState.NEUTRAL; }
                break;
            case NEUTRAL:
                gamepad2.stopRumble();
                if (gamepad2.triangle) {
                    arm.setTargetInchesRobotCentric(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION + 0.5);
                } else if (gamepad2.square) {
                    arm.setTargetInchesRobotCentric(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                    armState = ArmState.SUB;
                } else if (gamepad2.dpad_left) {
                    arm.setTargetInchesRobotCentric(GROUND_HORIZONTAL_POSITION, GROUND_VERTICAL_POSITION);
                } else if (gamepad2.circle) {
                    arm.setTargetInches(9.8, 6.0);
                } else if (gamepad2.cross) {
                    arm.setTargetInchesRobotCentric(-3.0, 20.0);
                } else if (gamepad2.dpad_up) {
                    arm.setTargetInches(9.8, 8.5);
                } else if (gamepad2.dpad_down) {
                    arm.setTargetInchesRobotCentric(1.5, 18.0);
                } else {
                    double gamepadleftX = gamepad2.left_stick_x;
                    double gamepadleftY = -gamepad2.left_stick_y;

                    if (Math.abs(gamepadleftX) < 0.1) gamepadleftX = 0.0;
                    if (Math.abs(gamepadleftY) < 0.2) gamepadleftY = 0.0;

                    arm.manualControlCartesian(gamepadleftX, gamepadleftY, 10.0);
                }
                break;
            case SUB:
                gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);

                if (arm.verticalInches() < 1.6) {
                    arm.setIntakePosition(INTAKE_OPEN_POSITION);
                } else {
                    arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                }

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setTargetInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    armState = ArmState.NEUTRAL;
                    arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                } else {
                    if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                        arm.setVerticalTargetInches(1);
                    } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        arm.setVerticalTargetInches(-4);
                    } else {
                        double xInput = -gamepad1.right_stick_y;

                        if (Math.abs(xInput) > 0.1) {
                            arm.manualControlSub(-gamepad1.right_stick_y, 5);
                        }
                    }

                }
                break;
        }

        armDebug.intake();
        armDebug.position();
        armDebug.cartesianPosition();
        arm.update();
    }

    private enum ArmState {
        HOMING,
        NEUTRAL,
        SUB
    }
}
