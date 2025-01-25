package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.auto.actions.HomeArmAction;
import org.firstinspires.ftc.teamcode.opmodes.auto.actions.MoveArmToPositionAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@Autonomous
public class Bar extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(8, -62, Math.PI / 2);

    private final double HIGH_BAR_VERTICAL_INCHES = 29.5;
    private final double HIGH_BAR_HORIZONTAL_INCHES = 3.0;
    private final double CLIPPING_VERTICAL_INCHES = 22.5;
    private final double WALL_VERTICAL_INCHES = 10.5;
    private final double WALL_HORIZONTAL_INCHES = 1.0;
    private final double PICKUP_VERTICAL_INCHES = 8.5;

    private Arm arm;

    @Override public void runOpMode() {
        arm = new Arm(hardwareMap);
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap)
                .setPose(initialPose)
                .build();

        TrajectoryActionBuilder startToBarAction = drive.actionBuilder(initialPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(4, -27), Math.PI/2);

        TrajectoryActionBuilder pushFirstTwoSamplesIntoToObservationZoneAction = startToBarAction.endTrajectory().fresh()
                // To First Sample
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(26, -36, Math.PI), Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(30, -21, -Math.PI /2), Math.PI/2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -10), 0)
                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(47, -56), -Math.PI / 2)
                // Push Sample Into Zone
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(47, -20, -Math.PI /2), Math.PI / 4)
                .setTangent(Math.PI/4)
                .splineToLinearHeading(new Pose2d(50, -16, -Math.PI /4), Math.PI / 2)
                // Push Second Sample Into Zone
                .setTangent(-Math.PI/4)
                .splineTo(new Vector2d(56, -21), -Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI /2), -Math.PI / 2);

        TrajectoryActionBuilder attachToFirstSample = pushFirstTwoSamplesIntoToObservationZoneAction.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-68, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder firstSamplePickupToBar = attachToFirstSample.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-67, null, new ProfileAccelConstraint(-70.0, 70.0))
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-2, -29, Math.PI /1.999), Math.PI / 2);

        TrajectoryActionBuilder barToSecondSamplePickup = firstSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(58, -56, -Math.PI / 2), -Math.PI / 2);

        TrajectoryActionBuilder attachToSecondSample = barToSecondSamplePickup.endTrajectory().fresh()
                .waitSeconds(0.3)
                .setTangent(-Math.PI/2)
                .lineToY(-71, null, new ProfileAccelConstraint(-70.0, 70.0));

        TrajectoryActionBuilder secondSamplePickupToBar = attachToSecondSample.endTrajectory().fresh()
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-0.5, -31, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bookItToObservationZoneBuilder = secondSamplePickupToBar.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(30, -60), 0)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(60, -66), 0);

        Action moveArmToHighBarAction
                = new MoveArmToPositionAction(arm, HIGH_BAR_VERTICAL_INCHES, HIGH_BAR_HORIZONTAL_INCHES);
        Action clipOnHighBarAction
                = new MoveArmToPositionAction(arm, CLIPPING_VERTICAL_INCHES, HIGH_BAR_HORIZONTAL_INCHES);
        Action moveArmToWallPickupPositionAction
                = new MoveArmToPositionAction(arm, WALL_VERTICAL_INCHES, WALL_HORIZONTAL_INCHES);
        Action pickupFromWallAction
                = new MoveArmToPositionAction(arm, PICKUP_VERTICAL_INCHES, WALL_HORIZONTAL_INCHES);

        waitForStart();

        homeArm();

        runArmToPosition(HIGH_BAR_HORIZONTAL_INCHES, HIGH_BAR_VERTICAL_INCHES, 1.0, 1.2);

        Actions.runBlocking(startToBarAction.build());
        retractArm(0.7);

        Actions.runBlocking(pushFirstTwoSamplesIntoToObservationZoneAction.build());
        runArmToPosition(WALL_HORIZONTAL_INCHES, WALL_VERTICAL_INCHES, 1.0, 2.0);

        arm.setIntakePosition(0.1);
        Actions.runBlocking(attachToFirstSample.build());
        arm.setIntakePosition(0.0);
        runArmToPosition(HIGH_BAR_HORIZONTAL_INCHES, HIGH_BAR_VERTICAL_INCHES, 1.0, 1.0);

        Actions.runBlocking(firstSamplePickupToBar.build());
        retractArm(0.7);

        Actions.runBlocking(barToSecondSamplePickup.build());
        runArmToPosition(WALL_HORIZONTAL_INCHES, WALL_VERTICAL_INCHES, 1.0, 2.0);

        arm.setIntakePosition(0.1);
        Actions.runBlocking(attachToSecondSample.build());
        arm.setIntakePosition(0.0);
        runArmToPosition(HIGH_BAR_HORIZONTAL_INCHES, HIGH_BAR_VERTICAL_INCHES, 1.0, 1.0);

        Actions.runBlocking(secondSamplePickupToBar.build());
        retractArm(0.65);
        Actions.runBlocking(bookItToObservationZoneBuilder.build());
    }

    private void homeArm() {
        while (!(arm.state() == Arm.ArmState.POSITION)) {
            arm.update();
        }
    }

    private void runArmToPosition(double horizontalInches, double veritcalInches, double maxPower, double timeout) {
        arm.setPositionMode();
        arm.setTargetInchesRobotCentric(horizontalInches, veritcalInches);
        arm.setMaxPower(maxPower, maxPower);

        ElapsedTime timer = new ElapsedTime();

        while (!arm.isAtPosition() && timer.seconds() < timeout) {
            arm.update();
        }

    }

    private void retractArm(double seconds) {
        arm.setManualMode();
        arm.manualControl(0.0, -0.7);

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < seconds) {
            arm.update();
        }

        arm.stop();
    }
}
