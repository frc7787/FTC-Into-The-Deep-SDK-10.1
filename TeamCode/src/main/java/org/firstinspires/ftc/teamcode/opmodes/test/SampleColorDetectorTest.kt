package org.firstinspires.ftc.teamcode.opmodes.test

import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utility.PIDController
import org.firstinspires.ftc.teamcode.vision.SampleColor.RED
import org.firstinspires.ftc.teamcode.vision.SampleColorDetector
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT

@TeleOp(name = "Test - Color Detector", group = "Test")
class SampleColorDetectorTest: CommandOpMode() {
    private val KP = 0.004
    private val KD = 0.000014

    private lateinit var camera: OpenCvCamera
    private lateinit var sampleDetector: SampleColorDetector
    private lateinit var tankDrive: TankDriveSubsystem
    private lateinit var turnController: PIDController

    override fun initialize() {
        sampleDetector  = SampleColorDetector()
        tankDrive      = TankDriveSubsystem(this)
        turnController = PIDController(KP, 0.0, KD)

        camera = initializeWebcam()

        register(tankDrive)

    }

    private fun initializeWebcam(): OpenCvCamera {
        val cameraMonitorViewId = hardwareMap
            .appContext
            .resources
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)

        val camera = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        camera.openCameraDeviceAsync(object: AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(320, 240, UPRIGHT)
                camera.setPipeline(sampleDetector)
            }

            override fun onError(errorCode: Int) {
                telemetry.addData("Error Opening Camera", errorCode)
            }
        })

        return camera
    }

    private fun centerOnLargestDetection() {
        val largestDetection = sampleDetector.getLargestDetection(RED)

        if (largestDetection == null) return

        if (largestDetection.rect.area() == 0.0) {
            telemetry.addLine("No Samples Detected")
            tankDrive.arcade(0.0, 0.0)
            return
        }

        val centerPoint = (largestDetection.rect.x + (largestDetection.rect.width) / 2.0).toInt()
        val distanceFromCenter = centerPoint - 160

        val power = -turnController.calculate(distanceFromCenter.toDouble(), 0.0)

        if (centerPoint >= (320 * 0.45) && centerPoint <= (320 * 0.6)) {
            telemetry.addLine("Centered")
            tankDrive.arcade(0.0, 0.0)
        }

        tankDrive.arcade(0.0, power)

        telemetry.addData("Distance From Center", distanceFromCenter)
        telemetry.addData("Center Point", centerPoint)
        telemetry.addData("Power", power)
    }
}