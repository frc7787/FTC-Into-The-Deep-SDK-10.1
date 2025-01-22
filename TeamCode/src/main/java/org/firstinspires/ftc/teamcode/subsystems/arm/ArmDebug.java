package org.firstinspires.ftc.teamcode.subsystems.arm;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public final class ArmDebug {
    private final Arm arm;
    private final Telemetry telemetry;

    public ArmDebug(@NonNull Arm arm, @NonNull Telemetry telemetry) {
        this.arm = arm;
        this.telemetry = telemetry;
    }

    /**
     * Displays debug information about the global state of the robot including armState and limit
     * switch status
     */
    public void global() {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData(
                "Front Rotation Limit Switch Pressed", arm.frontRotationLimitSwitch.isPressed());
        telemetry.addData(
                "Back Rotation Limit Switch Pressed", arm.backRotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", arm.extensionLimitSwitch.isPressed());
        telemetry.addData("Arm State", arm.armState);
        telemetry.addData("Homing State", arm.homingState);
        telemetry.addData("Extension Power", arm.leaderExtensionMotor.getPower());
        telemetry.addData("Rotation Power", arm.rotationMotor.getPower());
    }

    /**
     * Displays information about the polar coordinates of the robot
     */
    public void polar() {
        telemetry.addLine("----- Extension -----");
        telemetry.addData("Position", arm.leaderExtensionMotor.getCurrentPosition());
        telemetry.addData("Target Position", arm.extensionTargetPosition);
        telemetry.addData("Inches", arm.inches());
        telemetry.addData("Target Inches", arm.extensionTargetInches());
        telemetry.addData("At Position", arm.extensionAtPosition());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", arm.rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position", arm.rotationTargetPosition);
        telemetry.addData("Degrees", arm.degrees());
        telemetry.addData("Target Degrees", arm.rotationTargetDegrees());
        telemetry.addData("At Position", arm.rotationAtPosition());
    }

    /**
     * Displays debug information about the current of the arm.
     */
    public void current() {
        double extensionCurrent = arm.leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
                + arm.followerExtensionMotor.getCurrent(CurrentUnit.AMPS);
        double rotationCurrent = arm.rotationMotor.getCurrent(CurrentUnit.AMPS);

        telemetry.addLine("----- Current (AMPS) -----");
        telemetry.addData("Extension", extensionCurrent);
        telemetry.addData("Rotation", rotationCurrent);
        telemetry.addData("Total", extensionCurrent + rotationCurrent);
    }

    /**
     * Displays debug information about the cartesian position of the arm.
     */
    public void cartesian() {
        telemetry.addLine("----- Cartesian Position -----");
        telemetry.addLine("--- Arm Centric ---");
        telemetry.addData("X Inches", arm.horizontalInches());
        telemetry.addData("X Target Inches", arm.horizontalTargetInches());
        telemetry.addData("Y Inches", arm.verticalInches());
        telemetry.addData("Y Target Inches", arm.verticalTargetInches());
        telemetry.addLine("--- Robot Centric ---");
        telemetry.addData("X Inches", arm.horizontalInchesRobotCentric());
        telemetry.addData("X Target Inches", arm.horizontalTargetInchesRobotCentric());
        telemetry.addData("Y Inches", arm.verticalTargetInches());
        telemetry.addData("Y Target Inches", arm.verticalTargetInchesRobotCentric());
    }

    /**
     * Displays debug information about the intake.
     */
    public void intake() {
        telemetry.addLine("----- Intake Debug -----");
        telemetry.addData("Position", arm.intakeServo.getPosition());
    }
}
