package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmConversions.*;

public class Arm {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    final DcMotorImplEx leaderExtensionMotor,
                                followerExtensionMotor,
                                rotationMotor;

    final RevTouchSensor frontRotationLimitSwitch,
                                 backRotationLimitSwitch,
                                 extensionLimitSwitch;

    final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // Global State
    // ---------------------------------------------------------------------------------------------

    ArmState armState;
    HomingState homingState;

    int rotationTargetPosition, extensionTargetPosition;
    int rotationPosition, extensionPosition;
    double verticalTargetInches, horizontalTargetInches;

    double maxSpeed;

    boolean isFirstManualControlIteration;

    private ElapsedTime manualControlTimer;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.00135, 0, 0.0001);

    private final PIDController rotationController
            = new PIDController(0.0092, 0, 0.000012);

    // ---------------------------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------------------------

    public Arm(@NonNull HardwareMap hardwareMap) {

        leaderExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "leaderExtensionMotor");
        followerExtensionMotor = hardwareMap.get(DcMotorImplEx.class, "followerExtensionMotor");
        rotationMotor = hardwareMap.get(DcMotorImplEx.class, "rotationMotor");

        MotorUtility.setZeroPowerBehaviours(
                BRAKE, leaderExtensionMotor, followerExtensionMotor, rotationMotor);
        MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);

        frontRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "frontRotationLimitSwitch");
        backRotationLimitSwitch = hardwareMap.get(RevTouchSensor.class, "backRotationLimitSwitch");
        extensionLimitSwitch = hardwareMap.get(RevTouchSensor.class, "extensionLimitSwitch");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.0);
        intakeServo.setDirection(Servo.Direction.FORWARD);

        extensionTargetPosition = 0;
        rotationTargetPosition  = 0;

        maxSpeed = MAX_EXTENSION_POWER;

        horizontalTargetInches = START_POSITION_XY[0];
        verticalTargetInches = START_POSITION_XY[1];

        armState = ArmState.HOMING;
        homingState = HomingState.START;
        isFirstManualControlIteration = true;
        manualControlTimer = new ElapsedTime();
        manualControlTimer.reset();
    }

    // ---------------------------------------------------------------------------------------------
    // Core
    // ---------------------------------------------------------------------------------------------

    public void update() {
        rotationPosition = rotationMotor.getCurrentPosition();
        extensionPosition = leaderExtensionMotor.getCurrentPosition();

        switch (armState) {
            case HOMING:
                home();
                break;
            case NORMAL:
                double rotationPower
                        = rotationController.calculate(rotationPosition, rotationTargetPosition);
                rotationPower = Range.clip(rotationPower, MIN_ROTATION_POWER, MAX_ROTATION_POWER);
                double extensionPower
                        = extensionController.calculate(extensionPosition, extensionTargetPosition);
                extensionPower = Range.clip(extensionPower, MIN_EXTENSION_POWER, maxSpeed);

                if (extensionTargetPosition <= 0 && extensionLimitSwitch.isPressed()) {
                    extensionPower = 0.0;
                }

                if (rotationTargetPosition <= 0 && frontRotationLimitSwitch.isPressed()) {
                    rotationPower = 0.0;
                }

                if (backRotationLimitSwitch.isPressed() && rotationPower >= 0.0) {
                    rotationPower = 0.0;
                }

                if (-0.1 < rotationPower && rotationPower < 0.1 && rotationAtPosition()) {
                    rotationPower = 0.0;
                }

                if (-0.2 < extensionPower && extensionPower < 0.15 && extensionAtPosition()) {
                    extensionPower = 0.0;
                }

                if (rotationAtPosition()) rotationPower = 0.0;
                if (extensionAtPosition()) extensionPower = 0.0;

                leaderExtensionMotor.setPower(extensionPower);
                followerExtensionMotor.setPower(extensionPower);
                rotationMotor.setPower(rotationPower);
                break;
        }
    }

    public void stop() {
        rotationMotor.setPower(0);
        followerExtensionMotor.setPower(0);
        leaderExtensionMotor.setPower(0);
    }

    /**
     * Calculates the vertical position (in inches) required to keep the arm hovering just above
     * the samples given the supplied horizontal position (in inches)
     * @param xInches The horizontal position to calculate the vertical position for
     * @return The calculated vertical position
     */
    private static double calculatePolynomialRegression(double xInches) {
        return -10.2 + (2.02 * xInches) + (-0.149 * Math.pow(xInches, 2)) +
                (4.82e-3 * Math.pow(xInches, 3)) + (-5.58e-5 * Math.pow(xInches, 4));
    }

    /**
     * Manual control flavour using a polynomial regression to keep the intake level while
     * extending.
     * @param xInput The value to control the horizontal extension of the arm
     * @param speed The speed in in/sec to move the arm at. Increasing this value will make the arm
     *              move faster, however it will also increase its choppiness.
     */
    public void manualControlSub(double xInput, double speed) {
        horizontalTargetInches += (xInput * manualControlTimer.seconds() * speed);
        manualControlTimer.reset();
        horizontalTargetInches = Range.clip(horizontalTargetInches, 2.0, 30.0);
        verticalTargetInches = calculatePolynomialRegression(horizontalTargetInches);
        double[] polarCoordinates = cartesianToPolar(horizontalTargetInches, verticalTargetInches);
        rotationTargetPosition = rotationDegreesToTicksCorrected(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    /**
     * Manual control flavour allowing for control of horizontal and vertical position of the arm.
     * @param xInput Input to control the horizontal position of the arm. The regression can be
     *               found at <a href="https://docs.google.com/spreadsheets/d/1GnF52eFFj6NZARqxjOI01XXZJkzJ-WH21ysyxr2HlAs/edit?gid=0#gid=0"></a>
     * @param yInput Input to control the vertical position of the arm
     * @param inchesPerSecond The speed to move the arm at in in/sec. Increasing this value will increase the
     *              speed at which the arm extends, however it will also increase its choppiness.
     */
    public void manualControlCartesian(double xInput, double yInput, double inchesPerSecond) {
        horizontalTargetInches += (xInput * manualControlTimer.seconds() * inchesPerSecond);
        verticalTargetInches += (yInput * manualControlTimer.seconds() * inchesPerSecond);
        manualControlTimer.reset();
        if (horizontalTargetInches > 30) horizontalTargetInches = 30;
        if (verticalTargetInches < -5) verticalTargetInches = -5;
        double[] polarCoordinates = cartesianToPolar(horizontalTargetInches, verticalTargetInches);
        rotationTargetPosition = rotationDegreesToTicksCorrected(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    /**
     * Runs the homing sequence of the arm. Note this function is non blocking and must be called
     * continually until homingState == HomingState.COMPLETE
     */
    private void home() {
        switch (homingState) {
            case START:
                zeroIntake();
                if (extensionLimitSwitch.isPressed() && frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor, rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    homingState = HomingState.INITIAL_RETRACTION;
                }
                break;
            case INITIAL_RETRACTION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(leaderExtensionMotor, followerExtensionMotor);
                    homingState = HomingState.SAFETY_EXTENSION;
                } else {
                    leaderExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                    followerExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                }
                break;
            case SAFETY_EXTENSION:
                int currentPosition = leaderExtensionMotor.getCurrentPosition();
                double power = extensionController.calculate(currentPosition, 300);
                leaderExtensionMotor.setPower(power);
                followerExtensionMotor.setPower(power);

                if (Math.abs(currentPosition - 300) <= 10) {
                    leaderExtensionMotor.setPower(0);
                    followerExtensionMotor.setPower(0);
                    homingState = HomingState.HOMING_ROTATION;
                }
            case HOMING_ROTATION:
                if (frontRotationLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.FINAL_RETRACTION;
                } else {
                    rotationMotor.setPower(ROTATION_HOMING_POWER);
                }
                break;
            case FINAL_RETRACTION:
                if (extensionLimitSwitch.isPressed()) {
                    MotorUtility.reset(rotationMotor);
                    homingState = HomingState.COMPLETE;
                } else {
                    leaderExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                    followerExtensionMotor.setPower(EXTENSION_HOMING_POWER);
                }
                break;
            case COMPLETE:
                armState = ArmState.NORMAL;
                rotationTargetPosition  = 0;
                extensionTargetPosition = 0;
                MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                break;
        }
    }

    public void setTargetInches(double horizontalTargetInches, double verticalTargetInches) {
        if (horizontalTargetInches >= 30.0) horizontalTargetInches = 30.0;
        this.horizontalTargetInches = horizontalTargetInches;
        this.verticalTargetInches = verticalTargetInches;
        double[] polarCoordinates = cartesianToPolar(horizontalTargetInches,verticalTargetInches);
        rotationTargetPosition = rotationDegreesToTicksCorrected(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    public void setHorizontalTargetInches(double horizontalTargetInches) {
        setTargetInches(horizontalTargetInches, this.verticalTargetInches);
    }

    public void setVerticalTargetInches(double verticalTargetInches) {
        setTargetInches(this.horizontalTargetInches, verticalTargetInches);
    }

    public void setTargetInchesRobotCentric(double horizontalTargetInches, double verticalTargetInches) {
        setTargetInches(
                horizontalTargetInches + ROTATION_X_OFFSET_INCHES,
                verticalTargetInches + ROTATION_Y_OFFSET_INCHES
        );
    }

    public void setHorizontalTargetInchesRobotCentric(double horizontalTargetInches) {
        setTargetInches(horizontalTargetInches + ROTATION_X_OFFSET_INCHES, this.verticalTargetInches);
    }

    public void setVerticalTargetInchesRobotCentric(double verticalTargetInches) {
        setTargetInches(this.horizontalTargetInches, verticalTargetInches + ROTATION_Y_OFFSET_INCHES);
    }

    public void setIntakePosition(double position) {
        intakeServo.setPosition(position);
    }

    public void zeroIntake() { intakeServo.setPosition(0.0); }

    public void setMaxSpeed(double speed) {
        maxSpeed = Range.clip(speed, 0.0, 1.0);
    }

    public void setPowersManual(double extensionPower, double rotationPower) {
        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    public ArmState state() { return armState; }

    public double horizontalTargetInches() {
        return horizontalTargetInches;
    }

    public double verticalTargetInches() {
        return verticalTargetInches;
    }

    public int rotationPosition() {
        return rotationMotor.getCurrentPosition();
    }

    public int extensionPosition() {
        return leaderExtensionMotor.getCurrentPosition();
    }

    public double degrees() {
        return rotationMotor.getCurrentPosition() / ROTATION_TICKS_PER_DEGREE ;
    }

    public double inches() {
        return (leaderExtensionMotor.getCurrentPosition() / EXTENSION_TICKS_PER_INCH);
    }

    public double horizontalInches() {
        return cartesianToPolar(degrees(), inches())[0];
    }

    public double verticalInches() {
        return polarToCartesian(degrees(), inches())[1];
    }

    public double intakePosition() {
        return intakeServo.getPosition();
    }

    public int rotationTargetPosition() { return rotationTargetPosition; }

    public int extensionTargetPosition() { return extensionTargetPosition; }

    public double targetDegrees() {
        return rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;
    }

    public double targetInches() {
        return extensionTargetPosition / EXTENSION_TICKS_PER_INCH;
    }

    public double amps() {
        return leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + followerExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + rotationMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean rotationAtPosition() {
        return Math.abs(rotationPosition - rotationTargetPosition) <= ROTATION_POSITION_THRESHOLD;
    }

    public boolean extensionAtPosition() {
        return extensionPosition <= EXTENSION_POSITION_THRESHOLD + extensionTargetPosition
                && extensionPosition >= -EXTENSION_NEGATIVE_THRESHOLD + extensionTargetPosition;
    }

    public boolean isAtPosition() { return rotationAtPosition() && extensionAtPosition(); }

    // ---------------------------------------------------------------------------------------------
    // State Enums
    // ---------------------------------------------------------------------------------------------

    public enum ArmState {
        HOMING,
        NORMAL,
    }

    public enum HomingState {
        START,
        INITIAL_RETRACTION,
        SAFETY_EXTENSION,
        HOMING_ROTATION,
        FINAL_RETRACTION,
        COMPLETE
    }
}
