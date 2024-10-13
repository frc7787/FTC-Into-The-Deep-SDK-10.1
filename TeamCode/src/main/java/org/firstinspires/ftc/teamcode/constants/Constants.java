package org.firstinspires.ftc.teamcode.constants;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public final class Constants {

    public static class FileConstants {
        @SuppressLint("sdCardPath")
        public static final String SD_CARD_PATH
                = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";

        public static final String CONSTANTS_FILE_PATH = SD_CARD_PATH + "Constants/";
        public static final String APRIL_TAG_LOG_FILE_PATH = SD_CARD_PATH + "AprilTagLogs/";
    }

    public static class DrivebaseConstants {
        public static String FRONT_LEFT_DRIVE_MOTOR_NAME  = "frontLeftDriveMotor";
        public static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "frontRightDriveMotor";
        public static String BACK_LEFT_DRIVE_MOTOR_NAME   = "backLeftDriveMotor";
        public static String BACK_RIGHT_DRIVE_MOTOR_NAME  = "backRightDriveMotor";

        public static String IMU_NAME = "imu";
        public static IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        public static double DRIVE_DEAD_ZONE  = 0.05;
        public static double STRAFE_DEAD_ZONE = 0.05;
        public static double TURN_DEAD_ZONE   = 0.05;
    }

    public static class TestConstants {
        public static Servo.Direction test = Servo.Direction.FORWARD;
    }

    public static class AprilTagConstants {
        public static int GAIN          = 0;
        public static int EXPOSURE_MS   = 2;
        public static int WHITE_BALANCE = 4000;
    }

    public static class VisionConstants {
        public static int ERODE_PASSES = 5;

        public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 0, 0);

        public static Scalar LOW_HSV_RANGE_BLUE  = new Scalar(97, 50, 0);
        public static Scalar HIGH_HSV_RANGE_BLUE = new Scalar(125, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_ONE  = new Scalar(160, 150, 0);
        public static Scalar HIGH_HSV_RANGE_RED_ONE = new Scalar(180, 255, 255);

        public static Scalar LOW_HSV_RANGE_RED_TWO  = new Scalar(0, 100, 0);
        public static Scalar HIGH_HSV_RANGE_RED_TWO = new Scalar(10, 255, 255);

        public static Scalar LOW_HSV_RANGE_YELLOW = new Scalar(20, 80, 0);
        public static Scalar HIGH_HSV_RANGE_YELLOW = new Scalar(30, 255, 255);

        public static final Point CV_ANCHOR        = new Point(-1, -1);
        public static final Scalar CV_BORDER_VALUE = new Scalar(-1);
        public static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;
    }
}
