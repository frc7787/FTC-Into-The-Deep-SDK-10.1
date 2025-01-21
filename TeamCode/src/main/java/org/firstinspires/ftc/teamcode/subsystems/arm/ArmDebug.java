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

    public void global() {
        telemetry.addLine("----- Debug Global -----");
        telemetry.addData(
                "Front Rotation Limit Switch Pressed", arm.frontRotationLimitSwitch.isPressed());
        telemetry.addData(
                "Back Rotation Limit Switch Pressed", arm.backRotationLimitSwitch.isPressed());
        telemetry.addData("Extension Limit Switch Pressed", arm.extensionLimitSwitch.isPressed());
        telemetry.addData("Arm State", arm.armState);
        telemetry.addData("Homing State", arm.homingState);
    }

    public void position() {
        telemetry.addLine("----- Extension -----");
        telemetry.addData("Position", arm.leaderExtensionMotor.getCurrentPosition());
        telemetry.addData("Target Position", arm.extensionTargetPosition);
        telemetry.addData("Inches", arm.inches());
        telemetry.addData("Target Inches", arm.extensionTargetInches());
        telemetry.addData("Power", arm.leaderExtensionMotor.getPower());
        telemetry.addData("At Position", arm.extensionAtPosition());
        telemetry.addLine("----- Rotation -----");
        telemetry.addData("Position", arm.rotationMotor.getCurrentPosition());
        telemetry.addData("Target Position", arm.rotationTargetPosition);
        telemetry.addData("Degrees", arm.degrees());
        telemetry.addData("Target Degrees", arm.rotationTargetDegrees());
        telemetry.addData("Power", arm.rotationMotor.getPower());
        telemetry.addData("At Position", arm.rotationAtPosition());
        telemetry.addData("Horizontal Target Inches", arm.horizontalTargetInches);
        telemetry.addData("Vertical Target Inches", arm.verticalTargetInches);
    }

    public void amps() {
        double extensionCurrent = arm.leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
                + arm.followerExtensionMotor.getCurrent(CurrentUnit.AMPS);
        double rotationCurrent = arm.rotationMotor.getCurrent(CurrentUnit.AMPS);

        telemetry.addLine("----- Current (AMPS) -----");
        telemetry.addData("Extension", extensionCurrent);
        telemetry.addData("Rotation", rotationCurrent);
        telemetry.addData("Total", extensionCurrent + rotationCurrent);
    }

    public void cartesianPosition() {
        double[] cartesianCoordinates = ArmConversions.polarToCartesian(arm.degrees(), arm.inches());

        telemetry.addLine("----- Cartesian Position -----");
        telemetry.addData("X Position", cartesianCoordinates[0]);
        telemetry.addData("X Target Position", arm.horizontalTargetInches);
        telemetry.addData("Y Position", cartesianCoordinates[1]);
        telemetry.addData("Y Target Position", arm.verticalTargetInches);
    }

    public void intake() {
        telemetry.addLine("----- Intake Debug -----");
        telemetry.addData("Position", arm.intakeServo.getPosition());
    }
}
