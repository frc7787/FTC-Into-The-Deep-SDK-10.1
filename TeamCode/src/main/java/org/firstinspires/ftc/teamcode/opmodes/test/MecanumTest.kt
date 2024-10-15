package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.mecanum.commands.RobotCentricCommand
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController

@TeleOp(name = "Test - Mecanum", group = "Test")
class MecanumTest: CommandOpMode() {

    override fun initialize() {
        val mecanumDrivebase = MecanumDriveSubsystem(this)
        val driverController = PlayStationController(gamepad1)

        register(mecanumDrivebase)

        schedule(
            RobotCentricCommand(
                mecanumDrivebase,
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getRightX,
                true
            )
        )
    }
}