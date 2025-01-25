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

public final class Arm {
    // ---------------------------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------------------------

    final DcMotorImplEx leaderExtensionMotor, followerExtensionMotor, rotationMotor;

    final RevTouchSensor frontRotationLimitSwitch, backRotationLimitSwitch, extensionLimitSwitch;

    final Servo intakeServo;

    // ---------------------------------------------------------------------------------------------
    // Global State
    // ---------------------------------------------------------------------------------------------

    ArmState armState;
    HomingState homingState;

    int rotationTargetPosition, extensionTargetPosition;
    int rotationPosition, extensionPosition;

    double extensionInches, rotationDegrees, extensionTargetInches, rotationTargetDegrees;

    double verticalTargetInches, horizontalTargetInches, verticalInches, horizontalInches;

    double manualExtensionPower, manualRotationPower;

    double maxExtensionPower, maxRotationPower;

    boolean isFirstManualControlIteration;

    private final ElapsedTime manualControlTimer;

    // ---------------------------------------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------------------------------------

    private final PIDController extensionController
            = new PIDController(0.0012, 0, 0.0001);

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

        manualExtensionPower = 0.0;
        manualRotationPower = 0.0;

        maxExtensionPower = MAX_EXTENSION_POWER;
        maxRotationPower = MAX_ROTATION_POWER;

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
        updatePositionInformation();

        double rotationPower = 0;
        double extensionPower = 0;

        switch (armState) {
            case HOMING:
                home();
                return;
            case MANUAL:
                rotationPower = manualRotationPower;
                extensionPower = manualExtensionPower;

                if (horizontalInchesRobotCentric() >= MAX_HORIZONTAL_INCHES_ROBOT_CENTRIC) {
                    if (extensionPower >= 0.0) extensionPower = 0.0;
                    if (rotationPower < 0.0) extensionPower = -1.0;
                }

                rotationTargetPosition = rotationPosition;
                extensionTargetPosition = extensionPosition;
                break;
            case POSITION:
                rotationPower = rotationController.calculate(rotationPosition, rotationTargetPosition);
                extensionPower = extensionController.calculate(extensionPosition, extensionTargetPosition);

                if (extensionTargetPosition <= 0 && extensionLimitSwitch.isPressed()) {
                    extensionPower = 0.0;
                }

                if (rotationTargetPosition <= 0 && frontRotationLimitSwitch.isPressed()) {
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

                if (isAtPosition()) {
                    maxExtensionPower = MAX_EXTENSION_POWER;
                    maxRotationPower = MAX_ROTATION_POWER;
                }

                if (inches() < 0.5 && extensionPower < 0.0) extensionPower = 0.0;

                break;
        }

        rotationPower = Range.clip(rotationPower, -maxRotationPower, maxRotationPower);
        extensionPower = Range.clip(extensionPower, -maxExtensionPower, maxExtensionPower);

        if (frontRotationLimitSwitch.isPressed() && rotationPower < 0.0) rotationPower = 0.0;
        if (backRotationLimitSwitch.isPressed() && rotationPower > 0.0) rotationPower = 0.0;
        if (extensionLimitSwitch.isPressed() && extensionPower <= 0.0) extensionPower = 0.0;

        setExtensionPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    /**
     * Updates the position information about the arm.
     */
    public void updatePositionInformation() {
        extensionPosition = leaderExtensionMotor.getCurrentPosition();
        rotationPosition = rotationMotor.getCurrentPosition();

        extensionInches = extensionPosition / EXTENSION_TICKS_PER_INCH;
        rotationDegrees = rotationPosition / ROTATION_TICKS_PER_DEGREE;

        extensionTargetInches = extensionTargetPosition / EXTENSION_TICKS_PER_INCH;
        rotationTargetDegrees = rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;

        double[] cartesianPosition
                = polarToCartesian(rotationDegrees, extensionInches);

        horizontalInches = cartesianPosition[0];
        verticalInches = cartesianPosition[1];

        double[] cartesianTargetPosition
                = polarToCartesian(rotationTargetDegrees, extensionTargetInches);

        horizontalTargetInches = cartesianTargetPosition[0];
        verticalTargetInches = cartesianTargetPosition[1];
    }

    /**
     * Force stops the arm, by setting the power of all the motors to be zero. Will get overwritten
     * by subsequent calls to {@link Arm#update()}.
     */
    public void stop() {
        rotationMotor.setPower(0);
        setExtensionPower(0.0);
    }

    /**
     * Calculates the vertical position (in inches) required to keep the arm hovering just above
     * the samples given the supplied horizontal position (in inches)
     * @param xInches The horizontal position to calculate the vertical position for
     * @return The calculated vertical position
     */
    private static double calculatePolynomialRegression(double xInches) {
        return -9.7 + (2.02 * xInches) + (-0.149 * Math.pow(xInches, 2)) +
               (4.82e-3 * Math.pow(xInches, 3)) + (-5.58e-5 * Math.pow(xInches, 4));
    }

    /**
     * Pseudo manual control using a polynomial regression to keep the intake level. Should only be
     * used while extending in the sub.
     * @param xInput The value to control the horizontal extension of the arm
     * @param speed The speed in in/sec to move the arm at. Increasing this value will make the arm
     *              move faster, however it will also increase its choppiness.
     */
    public void pseudoManualControlSub(double xInput, double speed) {
        horizontalTargetInches += (xInput * manualControlTimer.seconds() * speed);
        manualControlTimer.reset();
        horizontalTargetInches = Range.clip(horizontalTargetInches, 2.0, 30.0);
        verticalTargetInches = calculatePolynomialRegression(horizontalTargetInches);
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
                   setExtensionPower(EXTENSION_HOMING_POWER);
                }
                break;
            case SAFETY_EXTENSION:
                int currentPosition = leaderExtensionMotor.getCurrentPosition();
                double power = extensionController.calculate(currentPosition, 300);

                setExtensionPower(power);

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
                    setExtensionPower(EXTENSION_HOMING_POWER);
                }
                break;
            case COMPLETE:
                armState = ArmState.POSITION;
                rotationTargetPosition  = 0;
                extensionTargetPosition = 0;
                MotorUtility.reset(rotationMotor, leaderExtensionMotor, followerExtensionMotor);
                break;
        }
    }

    private double frictionRegression(@NonNull Arm.Direction direction) {
      return 0.0;
    }

    /**
     * Sets the arm to manual mode. Only works if the arm is in the {@link ArmState#POSITION} state.
     */
    public void setManualMode() {
       if (armState != ArmState.POSITION) return;
       armState = ArmState.MANUAL;
    }

    /**
     * Sets the arm to position mode. Only works if the arm is in the {@link ArmState#MANUAL} state.
     */
    public void setPositionMode() {
        if (armState != ArmState.MANUAL) return;
        armState = ArmState.POSITION;
    }

    /**
     * Manual control for the arm. The inputs to this function will be ignored if the arm state is
     * not {@link ArmState#MANUAL}. To set the arm to the manual state, call {@link Arm#setManualMode}.
     * @param rotationInput The power to give the rotation motor
     * @param extensionInput The power to give the extension motors
     */
    public void manualControl(double rotationInput, double extensionInput) {
        if (armState != ArmState.MANUAL) return;

        manualRotationPower = rotationInput;
        manualExtensionPower = extensionInput;
    }

    /**
     * Manual control for the extension of the arm. The input to this function will be ignored if
     * the arm state is not {@link ArmState#MANUAL}. To set the arm to the manual state, call
     * {@link Arm#setManualMode()}.
     * @param extensionInput The power to give the extension motors
     */
    public void manualControlExtension(double extensionInput) {
        manualControl(manualRotationPower, extensionInput);
    }

    /**
     * Manual control for the rotation of the arm. The input to this function will be ignored if
     * the arm state is not {@link ArmState#MANUAL}. To set the arm to the manual state, call
     * {@link Arm#setManualMode()}.
     * @param rotationInput The power to give the rotation motor
     */
    public void manualControlRotation(double rotationInput) {
        manualControl(rotationInput, manualExtensionPower);
    }

    /**
     * Sets the target inches relative to the rotation point of the robot, in this case the center
     * of the arm. If the state of the arm is manual the target position will be ignored.
     * @param horizontalTargetInches How many inches out to move the arm
     * @param verticalTargetInches How many inches up to move the arm
     */
    public void setTargetInches(double horizontalTargetInches, double verticalTargetInches) {
        if (armState == ArmState.MANUAL) return;

        if (horizontalTargetInches >= 30.0) horizontalTargetInches = 30.0;
        this.horizontalTargetInches = horizontalTargetInches;
        this.verticalTargetInches = verticalTargetInches;
        double[] polarCoordinates = cartesianToPolar(horizontalTargetInches, verticalTargetInches);
        rotationTargetPosition = rotationDegreesToTicksCorrected(polarCoordinates[0]);
        extensionTargetPosition = extensionInchesToTicks(polarCoordinates[1]);
    }

    /**
     * Sets how many inches the arm should go out. Maintains the current vertical target inches.
     * If the state of the arm is manual the target position will be ignored.
     * @param horizontalTargetInches How many inches the arm should go out
     */
    public void setHorizontalTargetInches(double horizontalTargetInches) {
        setTargetInches(horizontalTargetInches, this.verticalTargetInches);
    }

    /**
     * Sets how many inches the arm should go up. Maintains the current horizontal target inches.
     * If the state of the arm is manual the target position will be ignored.
     * @param verticalTargetInches How many inches the arm should go up
     */
    public void setVerticalTargetInches(double verticalTargetInches) {
        setTargetInches(this.horizontalTargetInches, verticalTargetInches);
    }

    public void setVerticalTargetInchesSketchy(double verticalTargetInches) {
        setTargetInches(horizontalInches, verticalTargetInches);
    }

    /**
     * Sets the target position of the arm relative to the ground right in front of the robot. If
     * the state of the arm is {@link ArmState#MANUAL} the target position will be ignored.
     * @param horizontalTargetInches How many inches to send the arm out
     * @param verticalTargetInches How many inches to send the arm up.
     */
    public void setTargetInchesRobotCentric(double horizontalTargetInches, double verticalTargetInches) {
        setTargetInches(
                horizontalTargetInches + ROTATION_X_OFFSET_INCHES,
                verticalTargetInches + ROTATION_Y_OFFSET_INCHES
        );
    }

    /**
     * Set how many inches the arm should go out relative to the front of the robot. If the state
     * of the arm is {@link ArmState#MANUAL} the target position will be ignored.
     * @param horizontalTargetInches How many inches the arm should go out
     */
    public void setHorizontalTargetInchesRobotCentric(double horizontalTargetInches) {
        setTargetInches(horizontalTargetInches + ROTATION_X_OFFSET_INCHES, this.verticalTargetInches);
    }

    /**
     * Sets how many inches the arm should go up relative to the ground. If the state of the arm is
     * {@link ArmState#MANUAL} the target position will be ignored.
     * @param verticalTargetInches How many inches the arm should go up
     */
    public void setVerticalTargetInchesRobotCentric(double verticalTargetInches) {
        setTargetInches(this.horizontalTargetInches, verticalTargetInches + ROTATION_Y_OFFSET_INCHES);
    }

    /**
     * Sets the position of the intake
     * @param position The position to set the intake
     */
    public void setIntakePosition(double position) {
        intakeServo.setPosition(position);
    }

    /**
     * Sets the intake to the zero position
     */
    public void zeroIntake() { intakeServo.setPosition(0.0); }

    /**
     * Sets the max speed for the next movement.
     * @param extensionPower The max power for the next movement of the arm
     */
    public void setMaxExtensionPower(double extensionPower) {
        maxExtensionPower = Range.clip(extensionPower, -MAX_EXTENSION_POWER, MAX_EXTENSION_POWER);
    }

    /**
     * Sets the max rotation speed for the next movement.
     * @param rotationPower The rotation power for the next movement of the arm
     */
    public void setMaxRotationPower(double rotationPower) {
        maxRotationPower = Range.clip(rotationPower, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }

    /**
     * Sets the max speeds for the next movement
     * @param rotationPower The rotation power for the next movement of the arm
     * @param extensionPower The extension power for the next movement
     */
    public void setMaxPower(double rotationPower, double extensionPower) {
        setMaxRotationPower(rotationPower);
        setMaxExtensionPower(extensionPower);
    }

    /**
     * Override the state machine and manually set power to the motors
     * @param extensionPower The power to give the extension motors
     * @param rotationPower The power to give the rotation motor
     */
    public void setPowersManual(double extensionPower, double rotationPower) {
        leaderExtensionMotor.setPower(extensionPower);
        followerExtensionMotor.setPower(extensionPower);
        rotationMotor.setPower(rotationPower);
    }

    /**
     * Utility function that sets the power to both of the extension motors, just so that we don't
     * ever forget accidentally.
     * @param power The power to set the extension motors
     */
    private void setExtensionPower(double power) {
       leaderExtensionMotor.setPower(power);
       followerExtensionMotor.setPower(power);
    }

    // ---------------------------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------------------------

    /**
     * @return The current state of the arm
     */
    public ArmState state() { return armState; }

    /**
     * @return The horizontal target inches, relative to the center of rotation of the arm
     */
    public double horizontalTargetInches() {
        return horizontalTargetInches;
    }

    /**
     * @return The horizontal target inches, relative to the front of the robot.
     */
    public double horizontalTargetInchesRobotCentric() {
        return horizontalTargetInches + ROTATION_X_OFFSET_INCHES;
    }

    /**
     * @return The vertical target inches, relative to the center of rotation of the arm
     */
    public double verticalTargetInches() { return verticalTargetInches; }

    /**
     * @return The vertical target inches, relative to the ground.
     */
    public double verticalTargetInchesRobotCentric() {
        return verticalTargetInches + ROTATION_Y_OFFSET_INCHES;
    }

    /**
     * @return The current position of the rotation motor
     */
    public int rotationPosition() { return rotationPosition; }

    /**
     * @return The current port of the leader extension motor
     */
    public int extensionPosition() {
        return extensionPosition;
    }

    /**
     * @return The current degrees of the arm
     */
    public double degrees() { return rotationDegrees; }

    /**
     * @return How many inches the arm is extended out
     */
    public double inches() { return extensionInches; }

    /**
     * @return How many inches the arm is out, relative to the center of rotation of the arm
     */
    public double horizontalInches() { return horizontalInches; }

    /**
     * @return How many inches the arm is out, relative to the front of the robot
     */
    public double horizontalInchesRobotCentric() {
        return horizontalInches + ROTATION_X_OFFSET_INCHES;
    }

    /**
     * @return How many inches the arm is up, relative to the center of rotation of the arm
     */
    public double verticalInches() { return verticalInches; }

    /**
     * @return How many inches the ram is up, relative to the ground.
     */
    public double verticalInchesRobotCentric() { return verticalInches + ROTATION_Y_OFFSET_INCHES; }

    /**
     * @return The position of the intake
     */
    public double intakePosition() { return intakeServo.getPosition(); }

    /**
     * @return The rotation target position (in ticks)
     */
    public int rotationTargetPosition() { return rotationTargetPosition; }

    /**
     * @return The extension target position (in ticks)
     */
    public int extensionTargetPosition() { return extensionTargetPosition; }

    /**
     * @return The target angle of the arm, in degrees
     */
    public double rotationTargetDegrees() {
        return rotationTargetPosition / ROTATION_TICKS_PER_DEGREE;
    }

    /**
     * @return The target extension of the arm, in inches
     */
    public double extensionTargetInches() { return extensionTargetInches; }

    /**
     * @return The total current draw of the arm motors. This includes both extension motors, and
     *         the rotation motor. 
     */
    public double currentAmps() {
        return leaderExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + followerExtensionMotor.getCurrent(CurrentUnit.AMPS)
               + rotationMotor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * @return Whether the rotation is withing the target tolerance.
     */
    public boolean rotationAtPosition() {
        return Math.abs(rotationPosition - rotationTargetPosition) <= ROTATION_POSITION_TOLERANCE;
    }

    /**
     * @return Whether the extension is within the target tolerance
     */
    public boolean extensionAtPosition() {
        return extensionPosition <= EXTENSION_POSITION_TOLERANCE + extensionTargetPosition
               && extensionPosition >= -EXTENSION_POSITION_NEGATIVE_TOLERANCE
               + extensionTargetPosition;
    }

    /**
     * @return Whether both the rotation and extension are within target tolerance.
     */
    public boolean isAtPosition() { return rotationAtPosition() && extensionAtPosition(); }

    // ---------------------------------------------------------------------------------------------
    // State Enums
    // ---------------------------------------------------------------------------------------------

    /**
     * Represents the current state of the arm.
     */
    public enum ArmState {
        HOMING,
        POSITION,
        MANUAL,
    }

    /**
     * Represents the current section of the homing sequence that the robot is in.
     */
    public enum HomingState {
        START,
        INITIAL_RETRACTION,
        SAFETY_EXTENSION,
        HOMING_ROTATION,
        FINAL_RETRACTION,
        COMPLETE
    }

    private enum Direction {
       FORWARD,
       REVERSE
    }
}
