package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.DrivebaseConstants.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

public final class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotorImplEx frontLeftMotor,
                                frontRightMotor,
                                backLeftMotor,
                                backRightMotor;

    private final Telemetry telemetry;
    private final DcMotorImplEx[] motors;

    private final IMU imu;

    /**
     * Constructs a new drive subsystems
     * @param opMode The opMode you are running ; To obtain the HardwareMap and Telemetry objects
     */
    public MecanumDriveSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        frontLeftMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        frontRightMotor = opMode.hardwareMap.get(DcMotorImplEx.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        backLeftMotor   = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_LEFT_DRIVE_MOTOR_NAME);
        backRightMotor  = opMode.hardwareMap.get(DcMotorImplEx.class, BACK_RIGHT_DRIVE_MOTOR_NAME);

        motors = new DcMotorImplEx[]{
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        MotorUtility.setDirections(REVERSE, frontLeftMotor, backLeftMotor);
        MotorUtility.setDirections(FORWARD, frontRightMotor, backRightMotor);

        MotorUtility.setMotorZeroPowerBehaviors(BRAKE, motors);
    }

    /**
     * Sets the zero power behavior of the robot to float
     */
    public void coast() {
        MotorUtility.setMotorZeroPowerBehaviors(FLOAT, motors);
    }

    /**
     * Drives the robot relative to itself
     * @param drive The drive value
     * @param strafe The strafe value
     * @param turn The turn value
     */
    public void robotCentric(double drive, double strafe, double turn) {
        if (Math.abs(drive)  < DRIVE_DEAD_ZONE)  drive  = 0.0;
        if (Math.abs(strafe) < STRAFE_DEAD_ZONE) strafe = 0.0;
        if (Math.abs(turn)   < TURN_DEAD_ZONE)   turn   = 0.0;

        double thetaRadians = StrictMath.atan2(drive, strafe);
        double power        = StrictMath.hypot(strafe, drive);

        double sin_theta = StrictMath.sin(thetaRadians - Math.PI / 4.0);
        double cos_theta = StrictMath.cos(thetaRadians - Math.PI / 4.0);

        double max = Math.max(Math.abs(cos_theta), Math.abs(sin_theta));

        double frontLeftPower  = power * cos_theta / max + turn;
        double frontRightPower = power * sin_theta / max - turn;
        double backLeftPower   = power * sin_theta / max + turn;
        double backRightPower  = power * cos_theta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Drives the robot relative to the field
     * @param drive The value to move forward
     * @param strafe The value to strafe
     * @param turn The value to turn
     */
    public void fieldCentric(double drive, double strafe, double turn) {
        double theta = Math.atan2(drive, strafe) - imu.getRobotYawPitchRollAngles().getYaw(RADIANS);
        double power = Math.hypot(strafe, drive);

        double sin_theta = Math.sin(Math.toRadians(theta) - Math.PI / 4.0);
        double cos_theta = Math.cos(Math.toRadians(theta) - Math.PI / 4.0);

        double max = Math.max(Math.abs(cos_theta), Math.abs(sin_theta));

        double frontLeftPower  = power * cos_theta / max + turn;
        double frontRightPower = power * sin_theta / max - turn;
        double backLeftPower   = power * sin_theta / max + turn;
        double backRightPower  = power * cos_theta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Displays the direction of the drive motors
     */
    public void directionDebug() {
        telemetry.addData("Front Left Direction", frontLeftMotor.getDirection());
        telemetry.addData("Front Right Direction", frontRightMotor.getDirection());
        telemetry.addData("Back Left Direction", backLeftMotor.getDirection());
        telemetry.addData("Back Right Direction", backRightMotor.getDirection());
    }

    /**
     * Displays the current, in amps, of the drive motors
     */
    public void currentDebug() {
        telemetry.addLine("Current Unit: Amps");
        telemetry.addData("Front Left Current", frontLeftMotor.getCurrent(AMPS));
        telemetry.addData("Front Right Current", frontRightMotor.getCurrent(AMPS));
        telemetry.addData("Back Left Current", backLeftMotor.getCurrent(AMPS));
        telemetry.addData("Back Right Current", backRightMotor.getCurrent(AMPS));
    }

    /**
     * Displays the power of the drive motors
     */
    public void powerDebug() {
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
    }

    /**
     * Displays the velocity, in degrees per second, of the drive motors
     */
    public void velocityDebug() {
        telemetry.addLine("Velocity Unit: Degrees/Seconds");
        telemetry.addData("Front Left Velocity", frontLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Front Right Velocity", frontRightMotor.getVelocity(DEGREES));
        telemetry.addData("Back Left Velocity", backLeftMotor.getVelocity(DEGREES));
        telemetry.addData("Back Right Velocity", backRightMotor.getVelocity(DEGREES));
    }

    public void zeroPowerBehaviourDebug() {
        telemetry.addData(
                "Front Left Zero Power Behavior", frontLeftMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Front Right Zero Power Behavior", frontRightMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Back Left Zero Power Behavior", backLeftMotor.getZeroPowerBehavior());
        telemetry.addData(
                "Back Right Zero Power Behavior", backRightMotor.getZeroPowerBehavior());
    }

    /**
     * Displays debug information about the drive base
     */
    public void fullDebug() {
        directionDebug();
        currentDebug();
        powerDebug();
        velocityDebug();
        zeroPowerBehaviourDebug();
    }

}
