package org.firstinspires.ftc.teamcode.opmodes.test

import com.outoftheboxrobotics.photoncore.Photon
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonAdvancedDcMotor
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@Photon
@TeleOp(name = "Test - Motor")
class MotorTest: OpMode() {
    private lateinit var motorOne: PhotonAdvancedDcMotor
    private lateinit var motorTwo: PhotonAdvancedDcMotor

    override fun init() {
        motorOne = hardwareMap.get(DcMotor::class.java, "motorOne") as PhotonAdvancedDcMotor
        motorTwo = hardwareMap.get(DcMotor::class.java, "motorTwo") as PhotonAdvancedDcMotor
        motorOne.cacheTolerance = 0.01
        motorTwo.cacheTolerance = 0.02
        motorOne.setMotorSetRefreshRate(50)
        motorTwo.setMotorSetRefreshRate(50)
    }

    override fun loop() {
        val motorPower = gamepad1.left_stick_y * -1.0
        motorOne.setPower(motorPower)
        motorTwo.setPower(motorPower)
    }
}