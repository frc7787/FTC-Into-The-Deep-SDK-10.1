package org.firstinspires.ftc.teamcode.utility.playstationcontroller.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.RGB

class ChangeLEDColorCommand(
    val gamepad: GamepadEx,
    val color: RGB,
    val duration: Int
): CommandBase() {

    override fun execute() {
        gamepad.gamepad.setLedColor(color.red, color.green, color.blue, duration)
    }
}