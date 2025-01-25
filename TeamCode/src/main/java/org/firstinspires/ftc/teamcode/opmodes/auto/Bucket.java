package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

@Autonomous
public class Bucket extends LinearOpMode {
    private final Pose2d initialPose = new Pose2d(-16.5, -62, -Math.PI / 2);
    private Arm arm;

    @Override public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive.Builder(hardwareMap).build();

        arm = new Arm(hardwareMap);

        TrajectoryActionBuilder firstBuilder = drive.actionBuilder(initialPose)
                // start position to sub for clipping
                .waitSeconds(5)
                .lineToY(-50)   // north a bit
                .setTangent(0)
                .lineToX(-4)     // west a bit, more into the center of sub
                .setTangent(Math.PI/2)
                .lineToY(-23);   // north to the sub

        TrajectoryActionBuilder secondBuilder = firstBuilder.endTrajectory().fresh()
                // sub to behind the left hand spike mark
                //.waitSeconds(1)     // placeholder for action: CLIP
                //.afterTime(2, elevator.ClipIt())
                .setTangent(Math.PI/2)
                .lineToY(-38)   // south to a midpoint
                .setTangent(0)
                .lineToX(-36)    // west to clear sub
                .setTangent(Math.PI/2)
                .lineToY(-14)    // north past the right hand spike mark
                .setTangent(0)
                .lineToX(-47)   // west to line up with right hand spike mark
                .setTangent(Math.PI/2)
                .lineToY(-62)   // south to push block
                .setTangent(Math.PI/2)
                .lineToY(-14)   // north past the middle spike mark
                .setTangent(0)
                .lineToX(-55)   // west to line up with middle spike mark
                .setTangent(Math.PI/2)
                .lineToY(-62)   // south to push block
                .setTangent(Math.PI/2)       //.turnTo(-Math.PI/2)
                .lineToY(-14)   // north past the left hand spike mark
                .setTangent(0)
                .lineToX(-63)   // west to line up with left hand spike mark
                .setTangent(Math.PI/2)
                .lineToY(-50)  // south to push block
                .lineToY(-45)   // go to intermediate point for launching into spline
                .splineToLinearHeading(new Pose2d(-22,-10,0),0);    // spline to sub for parking

        // east to line up with left hand spike mark


        TrajectoryActionBuilder thirdBuilder = secondBuilder.endTrajectory().fresh()
                // parallel with clipHome, push in left hand spike mark, backup
                // turn around, move in for specimen pickup after a small wait

                .setTangent(Math.PI/2)
                .lineToY(-60)   // south to push sample into zone
                .setTangent(Math.PI/2)
                .lineToY(-55)   // north, backup out of zone
                // new position of turn
                .turn(-Math.PI)      // spin around for gripper to face wall
                .setTangent(-Math.PI/2)
                .lineToY(-65)   // south to intermediate point, human player lines up specimen (was -62)
                .setTangent(-Math.PI/2)
                //.waitSeconds(2)
                .lineToY(-71.5,null,new ProfileAccelConstraint(-70.0,70.0));
        //.lineToY(-71);  // south to pickup specimen


        TrajectoryActionBuilder fourthBuilder = thirdBuilder.endTrajectory().fresh()
                // from pickup specimen to clipping
                .lineToY(-55)
                .setTangent(0)
                .lineToX(1)
                .turnTo(Math.PI/2,
                        new TurnConstraints(2*Math.PI/3,-2*Math.PI/3,2*Math.PI/3))
                .setTangent(Math.PI/2)
                .lineToY(-23);

        TrajectoryActionBuilder fifthBuilder = fourthBuilder.endTrajectory().fresh()
                // from pickup specimen to clipping   .setTangent(-Math.PI/2)
                .lineToY(-30) //-48
                .setTangent(Math.PI/6)  // 0
                .lineToX(48,null, new ProfileAccelConstraint(-70.0,70.0));

        TrajectoryActionBuilder extraBuilder = firstBuilder.endTrajectory().fresh()
                .waitSeconds(5)
                .lineToY(-48)
                .waitSeconds(3)
                .setTangent(0)
                .lineToX(0);

        Action first = firstBuilder.build();
        Action second = secondBuilder.build();

        waitForStart();
        elapsedTime.reset();

        homeArm();
        runArmToPosition(3, 29.5, 1.0, 1.2);

        Actions.runBlocking(first);
        retractArm(0.7);

        Actions.runBlocking(second);
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