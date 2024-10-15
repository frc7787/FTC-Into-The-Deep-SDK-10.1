package org.firstinspires.ftc.teamcode.subsystems.mecanum.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs

class RobotCentricCommand(
    val mecanumDriveSubsystem: MecanumDrive,
    val driveSupplier: DoubleSupplier,
    val strafeSupplier: DoubleSupplier,
    val turnSupplier: DoubleSupplier,
) : CommandBase() {

    init { addRequirements(mecanumDriveSubsystem) }

    override fun execute() {
        var driveValue = driveSupplier.asDouble
        var strafeValue = strafeSupplier.asDouble
        var turnValue = turnSupplier.asDouble

        if (abs(driveValue) < 0.05) driveValue = 0.0
        if (abs(strafeValue) < 0.05) strafeValue = 0.0
        if (abs(turnValue) < 0.05) turnValue = 0.0

        mecanumDriveSubsystem.robotCentric(driveValue, strafeValue, turnValue)
    }
}
