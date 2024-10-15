package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.IMU.Parameters
import kotlin.math.*

class MecanumDrive(opMode: OpMode): SubsystemBase() {
        val frontLeftMotor  = opMode.hardwareMap.get(DcMotorImplEx::class.java, "frontLeftMotor")
        val frontRightMotor = opMode.hardwareMap.get(DcMotorImplEx::class.java, "frontRightMotor")
        val backLeftMotor   = opMode.hardwareMap.get(DcMotorImplEx::class.java, "backLeftMotor")
        val backRightMotor  = opMode.hardwareMap.get(DcMotorImplEx::class.java, "backRightMotor")
        val imu             = opMode.hardwareMap.get(IMU::class.java, "imu")

        val motors = arrayOf(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)
        val telemetry = opMode.telemetry

        init {
            imu.initialize(Parameters(RevHubOrientationOnRobot(UP, FORWARD)))
            MotorUtility.setDirections(REVERSE, frontLeftMotor, backLeftMotor)
        }

        fun robotCentric(drive: Double, strafe: Double, turn: Double) {
            val thetaRadians = atan2(drive, strafe)
            val power = hypot(strafe, drive)

            val sin = sin(thetaRadians - PI / 4.0)
            val cos = cos(thetaRadians - PI / 4.0)

            val max = max(abs(sin), abs(cos))

            var frontLeftPower  = power * cos / max + turn;
            var frontRightPower = power * sin / max - turn;
            var backLeftPower   = power * sin / max + turn;
            var backRightPower  = power * cos / max - turn;

            val turnMagnitude = abs(turn)

            if (power + turnMagnitude > 1.0) {
                frontLeftPower /= power + turnMagnitude
                frontRightPower /= power + turnMagnitude
                backLeftPower /= power + turnMagnitude
                backRightPower /= power + turnMagnitude
            }

            frontLeftMotor.power  = frontLeftPower
            frontRightMotor.power = frontRightPower
            backLeftMotor.power   = backLeftPower
            backRightMotor.power  = backRightPower
        }

        fun fieldCentric(drive: Double, strafe: Double, turn: Double) {
            val thetaRadians = atan2(drive, strafe) - imu.robotYawPitchRollAngles.yaw
            val power = hypot(strafe, drive)

            val sin = sin(thetaRadians - PI / 4.0)
            val cos = cos(thetaRadians - PI / 4.0)

            val max = max(abs(sin), abs(cos))

            var frontLeftPower  = power * cos / max + turn;
            var frontRightPower = power * sin / max - turn;
            var backLeftPower   = power * sin / max + turn;
            var backRightPower  = power * cos / max - turn;

            val turnMagnitude = abs(turn)

            if (power + turnMagnitude > 1.0) {
                frontLeftPower /= power + turnMagnitude
                frontRightPower /= power + turnMagnitude
                backLeftPower /= power + turnMagnitude
                backRightPower /= power + turnMagnitude
            }

            frontLeftMotor.power  = frontLeftPower
            frontRightMotor.power = frontRightPower
            backLeftMotor.power   = backLeftPower
            backRightMotor.power  = backRightPower
        }
}