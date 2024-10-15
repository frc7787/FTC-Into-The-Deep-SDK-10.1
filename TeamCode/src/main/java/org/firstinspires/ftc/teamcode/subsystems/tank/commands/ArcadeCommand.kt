package org.firstinspires.ftc.teamcode.subsystems.tank.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.tank.TankDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs

class ArcadeCommand(
    val tankDrive: TankDrive,
    val driveSupplier: DoubleSupplier,
    val turnSupplier: DoubleSupplier
): CommandBase() {

    init { addRequirements(tankDrive) }

    override fun execute() {
        var driveValue = driveSupplier.asDouble
        var turnValue = turnSupplier.asDouble

        if (abs(driveValue) < 0.05) driveValue = 0.0
        if (abs(turnValue) < 0.05) turnValue = 0.0

        tankDrive.arcade(driveValue, turnValue)
    }
}