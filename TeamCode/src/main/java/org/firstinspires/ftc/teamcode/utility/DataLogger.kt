package org.firstinspires.ftc.teamcode.utility

import org.firstinspires.ftc.teamcode.constants.Constants.FileConstants
import java.io.*
import java.text.SimpleDateFormat
import java.util.*
import java.util.Locale.CANADA

class DataLogger(var logFileName: String) {
    private val data: ArrayList<String?> = ArrayList<String?>()

    fun addLine(line: String?) {
        data.add(line)
    }

    fun clear() {
        val heading = data[0]
        data.clear()
        data.add(heading)
    }

    fun save() {
        val currentDate = SimpleDateFormat("yyyyMMdd", CANADA).format(Date())
        val currentTime = SimpleDateFormat("HHmmss", CANADA).format(Date())

        logFileName += "${currentDate}_${currentTime}.txt"

        try {
            FileWriter("${FileConstants.SD_CARD_PATH}$logFileName").use { fileWriter ->
                for (data in data) fileWriter.write(data)
                fileWriter.flush()
            }
        } catch (_: IOException) { }
    }
}
