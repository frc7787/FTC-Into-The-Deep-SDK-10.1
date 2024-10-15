package org.firstinspires.ftc.teamcode.subsystems.tank.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.tank.TankDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs

class TankCommand(
    val tankDrive: TankDrive,
    val leftPowerSupplier: DoubleSupplier,
    val rightPowerSupplier: DoubleSupplier
): CommandBase() {

    init { addRequirements(tankDrive) }

    override fun execute() {
        var leftPower  = leftPowerSupplier.asDouble
        var rightPower = rightPowerSupplier.asDouble

        if (abs(leftPower) < 0.05) leftPower = 0.0
        if (abs(rightPower) < 0.05) rightPower = 0.0

        tankDrive.tank(leftPower, rightPower)
    }

}