package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointOdometry
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointOdometry.*

@TeleOp(name = "Test - OpMode")
class OdometryTest: CommandOpMode() {
    private lateinit var odometry: GoBildaPinpointOdometry

    override fun initialize() {
        odometry = hardwareMap.get(GoBildaPinpointOdometry::class.java, "odometry")

        initializeOdometry()

        schedule(
            RunCommand(this::displayOdometryPosition),
            RunCommand(telemetry::update)
        )
    }

    fun initializeOdometry() {
        odometry = hardwareMap.get(GoBildaPinpointOdometry::class.java, "odometry")
        odometry.setEncoderResolution(FOUR_BAR_POD_TICKS_PER_MM.toDouble())
        odometry.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.REVERSED)
        odometry.setOffsets(1200.0, 250.0)
        odometry.resetPosAndIMU()

        telemetry.addData("X Offset", odometry.xOffset)
        telemetry.addData("Y Offset", odometry.yOffset)
        telemetry.addData("Version", odometry.version)
        telemetry.update()
    }

    fun displayOdometryPosition() {
        telemetry.addData("X Position", odometry.posX)
        telemetry.addData("Y Position", odometry.posY)
        telemetry.addData("Heading", odometry.heading)
    }
}