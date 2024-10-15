package org.firstinspires.ftc.teamcode.subsystems.mecanum.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDrive
import java.util.function.DoubleSupplier
import kotlin.math.abs

class FieldCentricCommand(
    val mecanumDriveSubsystem: MecanumDrive,
    val driveSupplier: DoubleSupplier,
    val strafeSupplier: DoubleSupplier,
    val turnSupplier: DoubleSupplier
) : CommandBase() {

    init { addRequirements(mecanumDriveSubsystem) }

    override fun execute() {
        var drivePower = driveSupplier.asDouble
        var strafePower = strafeSupplier.asDouble
        var turnPower = turnSupplier.asDouble

        if (abs(drivePower) < 0.05) drivePower = 0.0
        if (abs(strafePower) < 0.05) strafePower = 0.0
        if (abs(turnPower) < 0.05) turnPower = 0.0

        mecanumDriveSubsystem.fieldCentric(drivePower, strafePower, turnPower)
    }
}
