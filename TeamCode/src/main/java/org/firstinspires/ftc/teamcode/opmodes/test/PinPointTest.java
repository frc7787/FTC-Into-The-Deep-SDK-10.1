package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointOdometry;

@TeleOp(name = "Test - Pin Point Sensor", group = "Test")
public final class PinPointTest extends OpMode {
    private GoBildaPinpointOdometry odometry;

    @Override public void init() {
        odometry = hardwareMap.get(GoBildaPinpointOdometry.class, "odometry");

        odometry.setEncoderResolution(GoBildaPinpointOdometry.FOUR_BAR_POD_TICKS_PER_MM);

        odometry.setEncoderDirections(
                GoBildaPinpointOdometry.EncoderDirection.REVERSED,
                GoBildaPinpointOdometry.EncoderDirection.REVERSED
        );

        odometry.setOffsets(1200, 250);

        odometry.resetPosAndIMU();

        telemetry.addData("X Offset", odometry.getXOffset());
        telemetry.addData("Y Offset", odometry.getYOffset());
        telemetry.addData("Version Number", odometry.getDeviceVersion());
        telemetry.update();
    }

    @Override public void loop() {
        odometry.update();
        telemetry.addData("X", odometry.getEncoderX());
        telemetry.addData("Y", odometry.getEncoderY());
        telemetry.addData("H", Math.toDegrees(odometry.getHeading()));
    }
}