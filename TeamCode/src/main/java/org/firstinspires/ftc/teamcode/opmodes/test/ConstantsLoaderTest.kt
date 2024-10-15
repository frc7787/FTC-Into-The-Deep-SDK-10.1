package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.constants.ConstantsLoader

class ConstantsLoaderTest: CommandOpMode() {

    override fun initialize() {
        ConstantsLoader(telemetry).load()
    }
}