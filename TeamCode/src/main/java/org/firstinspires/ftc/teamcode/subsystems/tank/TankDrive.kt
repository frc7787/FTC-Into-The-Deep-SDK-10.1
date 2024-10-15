package org.firstinspires.ftc.teamcode.subsystems.tank

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.IMU.Parameters
import com.qualcomm.robotcore.util.Range
import kotlin.math.*

class TankDrive(hardwareMap: HardwareMap): SubsystemBase() {
    val leftMotor  = hardwareMap.get(DcMotorImplEx::class.java, "leftMotor")
    val rightMotor = hardwareMap.get(DcMotorImplEx::class.java, "rightMotor")
    val imu        = hardwareMap.get(IMU::class.java, "imu")

    init {
        leftMotor.direction = REVERSE
        leftMotor.zeroPowerBehavior = BRAKE
        rightMotor.zeroPowerBehavior = BRAKE

        imu.initialize(Parameters(RevHubOrientationOnRobot(UP, FORWARD)))
        imu.resetYaw()
    }

    fun tank(leftPower: Double, rightPower: Double) {
        leftMotor.power  = leftPower
        rightMotor.power = rightPower
    }

    fun arcade(drive: Double, turn: Double) {
        leftMotor.power  = Range.clip(drive + turn, -1.0, 1.0)
        rightMotor.power = Range.clip(drive - turn, -1.0, 1.0)
    }

    fun curvature(forward: Double, reverse: Double, rotation: Double, turnOverride: Boolean) {
        val drive = forward + reverse

        var leftPower: Double
        var rightPower: Double

        if (turnOverride) {
            leftPower = drive + rotation
            rightPower = drive - rotation
        } else {
            leftPower = drive + abs(drive) * rotation;
            rightPower = drive -  abs(drive) * rotation;
        }

        val maxMagnitude = max(abs(leftPower), abs(rightPower))

        if (maxMagnitude > 1.0) {
            leftPower /= maxMagnitude
            rightPower /= maxMagnitude
        }

        leftMotor.power = leftPower
        rightMotor.power = rightPower
    }
}