package org.firstinspires.ftc.teamcode.utility

import com.qualcomm.robotcore.util.ElapsedTime

class PIDController(val KP: Double, val KI: Double, val KD: Double) {
    val timer = ElapsedTime()
    var isFirstIteration = true
    var lastError = 0.0
    var integralSum = 0.0

    fun calculate(currentPosition: Double, targetPosition: Double): Double {
        val error = targetPosition - currentPosition

        if (isFirstIteration) {
            lastError = error
            timer.reset()
            isFirstIteration = false
            return KP * error
        } else {
            val deltaTime = timer.seconds()
            val derivative = if (deltaTime != 0.0) (error - lastError) / deltaTime else 0.0
            val integralSum = integralSum + error * deltaTime
            timer.reset()
            return KP * error + KI * integralSum + KD * derivative
        }
    }

    fun reset() {
        isFirstIteration = true
        lastError = 0.0
        integralSum = 0.0
    }
}