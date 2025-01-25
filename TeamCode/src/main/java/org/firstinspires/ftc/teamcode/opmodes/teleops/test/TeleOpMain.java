package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private final double INTAKE_OPEN_POSITION = 0.55;
    private final double INTAKE_SUB_PRIMED_POSITION = 0.35;
    private final double INTAKE_CLOSED_POSITION = 0.00;
    private final double INTAKE_SPECIMEN_PICKUP_POSITION = 0.25;
    private final double INTAKE_CLIPPING_POSITION = 0.22;

    private final double BUCKET_VERTICAL_POSITION = 56;
    private final double BUCKET_HORIZONTAL_POSITION = 8.0;
    private final double BAR_VERTICAL_POSITION = 31;
    private final double BAR_HORIZONTAL_POSITION = 3.5;

    private final double SUB_VERTICAL_POSITION = 0.4;
    private final double SUB_HORIZONTAL_POSITION = 2.0;

    private final double NEUTRAL_VERTICAL_POSITION = 6.0;
    private final double NEUTRAL_HORIZONTAL_POSITION = 3.0;

    private final double GROUND_VERTICAL_POSITION = -2.0;
    private final double GROUND_HORIZONTAL_POSITION = 0.0;

    private final double HOVER_VERTICAL_POSITION = 0.3;

    private final double WALL_VERTICAL_INCHES = 10.5;
    private final double WALL_HORIZONTAL_INCHES = 1.0;

    private Gamepad previousGamepad2, currentGamepad2, previousGamepad1, currentGamepad1;

    private TeleOpState teleOpState;

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

        teleOpState = TeleOpState.NEUTRAL;
    }

    @Override public void loop() {
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        drive.drive(driveInput, strafeInput, turnInput);

        if (gamepad2.left_bumper) {
            arm.setIntakePosition(INTAKE_OPEN_POSITION);
        } else if (gamepad2.right_bumper) {
            arm.setIntakePosition(INTAKE_CLOSED_POSITION);
        }

        switch (teleOpState) {
            case NEUTRAL:
                gamepad2.stopRumble();

                double rotationInput = -gamepad2.right_stick_y;
                double extensionInput = -gamepad2.left_stick_y;

                if (Math.abs(rotationInput) > 0.0 || Math.abs(extensionInput) > 0.0) {
                    arm.setManualMode();
                    arm.manualControl(rotationInput, extensionInput);
                } else {
                    arm.setPositionMode();
                    if (gamepad2.triangle) {
                        arm.setTargetInchesRobotCentric(BUCKET_HORIZONTAL_POSITION, BUCKET_VERTICAL_POSITION + 0.5);
                    } else if (gamepad2.square) {
                        arm.setTargetInchesRobotCentric(BAR_HORIZONTAL_POSITION, BAR_VERTICAL_POSITION);
                    } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                        arm.setTargetInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                        arm.setIntakePosition(INTAKE_SUB_PRIMED_POSITION);
                        teleOpState = TeleOpState.SUB;
                    } else if (gamepad2.dpad_left) {
                        arm.setTargetInchesRobotCentric(GROUND_HORIZONTAL_POSITION, GROUND_VERTICAL_POSITION);
                    } else if (gamepad2.circle) {
                        arm.setTargetInchesRobotCentric(WALL_HORIZONTAL_INCHES, WALL_VERTICAL_INCHES + 1);
                    } else if (gamepad2.dpad_up) {
                        arm.setTargetInchesRobotCentric(WALL_HORIZONTAL_INCHES + 1, WALL_VERTICAL_INCHES + 4.5);
                    }
                }
                break;
            case SUB:
                gamepad2.rumble(0.1, 0.1, Gamepad.RUMBLE_DURATION_CONTINUOUS);

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    arm.setPositionMode();
                    arm.setTargetInchesRobotCentric(NEUTRAL_HORIZONTAL_POSITION, NEUTRAL_VERTICAL_POSITION);
                    teleOpState = TeleOpState.NEUTRAL;
                    arm.setIntakePosition(INTAKE_CLOSED_POSITION);
                } else {
                    if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                        arm.setPositionMode();
                        arm.setVerticalTargetInchesSketchy(1);
                        arm.setIntakePosition(0.0);
                    } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        arm.setPositionMode();
                        arm.setVerticalTargetInchesSketchy(-5);
                        arm.setIntakePosition(0.35);
                    } else {
                        double horizontalInput = -gamepad1.right_stick_y;

                        if (Math.abs(horizontalInput) > 0.1) {
                            arm.setManualMode();
                            arm.manualControl(0.0, horizontalInput);
                        } else {
                            arm.setPositionMode();
                        }
                    }
                }
                break;
        }

        armDebug.intake();
        armDebug.polar();
        armDebug.cartesian();
        arm.update();
    }

    private enum TeleOpState {
        NEUTRAL,
        SUB
    }
}
