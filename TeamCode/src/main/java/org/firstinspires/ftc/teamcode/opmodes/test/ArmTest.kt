package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.ArmSubsystem

@TeleOp(name = "Test - Arm")
class ArmTest: OpMode() {
    lateinit var arm: ArmSubsystem

    override fun init() {
        arm = ArmSubsystem(hardwareMap)
    }

    override fun loop() {
        arm.rotationPower  = gamepad1.left_stick_x.toDouble()
        arm.extensionPower = gamepad1.right_stick_x.toDouble()
    }
}