package org.firstinspires.ftc.teamcode.utility.playstationcontroller

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.qualcomm.robotcore.hardware.Gamepad

/**
 * Wrapper class around GamepadKeys.Button enum
 * to accurately represent PS4/PS5 bindings
 */
class PlayStationController(val gamepad: Gamepad) : GamepadEx(gamepad) {

    fun leftTrigger(): Double {
        return gamepad.left_trigger.toDouble()
    }

    fun rightTrigger(): Double {
        return gamepad.right_trigger.toDouble()
    }

    fun leftBumper(): Boolean {
        return gamepad.left_bumper
    }

    fun rightBumper(): Boolean {
        return gamepad.right_bumper
    }

    fun cross(): Boolean {
        return gamepad.cross
    }

    fun square(): Boolean {
        return gamepad.square
    }

    fun triangle(): Boolean {
        return gamepad.triangle
    }

    fun circle(): Boolean {
        return gamepad.circle
    }

    fun dpadUp(): Boolean {
        return gamepad.dpad_up
    }

    fun dpadDown(): Boolean {
        return gamepad.dpad_down
    }

    fun dpadLeft(): Boolean {
        return gamepad.dpad_left
    }

    fun dpadRight(): Boolean {
        return gamepad.dpad_right
    }

    companion object {
        val LEFT_BUMPER: Button = Button.LEFT_BUMPER
        val RIGHT_BUMPER: Button = Button.RIGHT_BUMPER
        val DPAD_UP: Button = Button.DPAD_UP
        val DPAD_DOWN: Button = Button.DPAD_DOWN
        val DPAD_LEFT: Button = Button.DPAD_LEFT
        val DPAD_RIGHT: Button = Button.DPAD_RIGHT
        val TRIANGLE: Button = Y
        val SQUARE: Button = X
        val CROSS: Button = A
        val CIRCLE: Button = B
        val SHARE: Button = BACK
        val OPTIONS: Button = START
    }
}
