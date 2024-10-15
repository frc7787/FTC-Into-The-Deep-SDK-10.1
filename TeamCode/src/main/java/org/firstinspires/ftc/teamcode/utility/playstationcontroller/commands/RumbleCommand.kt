package org.firstinspires.ftc.teamcode.utility.playstationcontroller.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx

class RumbleCommand(
    val gamepad: GamepadEx,
    val duration: Int,
    val intensity: Double
): CommandBase() {

    override fun execute() {
        gamepad.gamepad.rumble(intensity, intensity, duration)
    }
}