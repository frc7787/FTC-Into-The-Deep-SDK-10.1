package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController

@TeleOp(name = "Test - Tank", group = "Test")
class TankTest: CommandOpMode() {

    override fun initialize() {
        val tankDrive = TankDriveSubsystem(this)
        val driverController = PlayStationController(gamepad1)

        register(tankDrive)

        schedule(
            TankCommand(
                tankDrive,
                driverController::getLeftX,
                driverController::getLeftY,
                false
            )
        )
    }
}