package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.ConstantsSaver

@TeleOp(name = "Test - Constants Saver", group = "Test")
class ConstantsSaverTest: CommandOpMode() {

    override fun initialize() {
        ConstantsSaver(telemetry).save()
    }
}