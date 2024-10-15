package org.firstinspires.ftc.teamcode.subsystems.tank.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.tank.TankDrive
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.abs

class CurvatureCommand(
    val tankDrive: TankDrive,
    val forwardPowerSupplier: DoubleSupplier,
    val reversePowerSupplier: DoubleSupplier,
    val turnPowerSupplier: DoubleSupplier,
    val turnOverrideSupplier: BooleanSupplier
): CommandBase() {

    init { addRequirements(tankDrive) }

    override fun execute() {
        var forwardPower = forwardPowerSupplier.asDouble
        var reversePower = -reversePowerSupplier.asDouble
        var turnPower = turnPowerSupplier.asDouble

        if (abs(forwardPower) < 0.05) forwardPower = 0.0
        if (abs(reversePower) < 0.05) reversePower = 0.0
        if (abs(turnPower) < 0.05) turnPower = 0.0

        tankDrive.curvature(forwardPower, reversePower, turnPower, turnOverrideSupplier.asBoolean)
    }
}