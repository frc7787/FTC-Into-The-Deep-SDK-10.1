package org.firstinspires.ftc.teamcode.utility

import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.ArrayList

open class Debugger(val telemetry: Telemetry?) {
    protected val output: ArrayList<String?> = ArrayList<String?>()

    fun addMessage(message: String) {
        output.add(message)
    }

    fun displayAll() {
        if (telemetry == null) return

        telemetry.addData("Number Of Issues", output.size)

        for (message in output) {
            telemetry.addLine("\n" + message)
        }

        telemetry.update()
    }
}


