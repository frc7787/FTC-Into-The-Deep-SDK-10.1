package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap

class ArmSubsystem(hardwareMap: HardwareMap): SubsystemBase() {
    private val rotationMotor = hardwareMap.get(Motor::class.java, "rotationMotor")
    private val extensionMotorGroup: MotorGroup

    var extensionPower = 0.0
    var rotationPower = 0.0

    init {
        val extensionMotorOne = hardwareMap.get(Motor::class.java, "extensionMotorOne")
        val extensionMotorTwo = hardwareMap.get(Motor::class.java, "extensionMotorTwo")
        extensionMotorGroup = MotorGroup(extensionMotorOne, extensionMotorTwo)
    }

    override fun periodic() {
        extensionMotorGroup.set(extensionPower)
        rotationMotor.set(rotationPower)
    }
}