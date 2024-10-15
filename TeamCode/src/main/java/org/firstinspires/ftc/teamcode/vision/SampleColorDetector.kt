package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.CV_ANCHOR
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.CV_BORDER_TYPE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.CV_BORDER_VALUE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.ERODE_PASSES
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.HIGH_HSV_RANGE_BLUE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.HIGH_HSV_RANGE_RED_ONE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.HIGH_HSV_RANGE_RED_TWO
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.HIGH_HSV_RANGE_YELLOW
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.LOW_HSV_RANGE_BLUE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.LOW_HSV_RANGE_RED_ONE
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.LOW_HSV_RANGE_RED_TWO
import org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.LOW_HSV_RANGE_YELLOW
import org.opencv.core.Core
import org.opencv.core.Core.BORDER_CONSTANT
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Rect
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.ArrayList

/**
 * OpenCV pipeline to detect the different colored samples for the FTC 2024-2025 game Into The Deep
 * @Author Tyler Stocks
 */
class SampleColorDetector : OpenCvPipeline() {
    private val hsvMat = Mat()
    private val threshold0 = Mat()
    private val threshold1 = Mat()
    private val hierarchy = Mat()
    private val cvErodeKernel = Mat()
    private val thresholdOutput = Mat()
    private val erodeOutput = Mat()
    private val dilateOutput = Mat()
    private val cvDilateKernel = Mat()

    private var sampleDetections = ArrayList<SampleDetection>()
    private val largestDetection: SampleDetection? = null
    private val largestRedSample: SampleDetection? = null
    private val largestYellowSample: SampleDetection? = null
    private val largestGreenSample: SampleDetection? = null

    override fun processFrame(input: Mat): Mat {
        sampleDetections = ArrayList<SampleDetection>()

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV)

        detectSample(input, SampleColor.RED)

        return input
    }

    @Synchronized
    private fun detectSample(input: Mat, color: SampleColor) {
        when (color) {
            SampleColor.RED -> {
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, threshold0)
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, threshold1)
                Core.add(threshold0, threshold1, thresholdOutput)
            }
            SampleColor.BLUE -> {
                Core.inRange(hsvMat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, thresholdOutput)
            }
            SampleColor.YELLOW -> {
                Core.inRange(hsvMat, LOW_HSV_RANGE_YELLOW, HIGH_HSV_RANGE_YELLOW, thresholdOutput)
            }
        }

        // Erode to remove noise
        Imgproc.erode(
            thresholdOutput,
            erodeOutput,
            cvErodeKernel,
            CV_ANCHOR,
            ERODE_PASSES,
            BORDER_CONSTANT,
            CV_BORDER_VALUE
        )

        Imgproc.dilate(
            erodeOutput,
            dilateOutput,
            cvDilateKernel,
            CV_ANCHOR,
            4,
            CV_BORDER_TYPE,
            CV_BORDER_VALUE
        )

        // Finds the contours of the image
        val contours = ArrayList<MatOfPoint?>()

        Imgproc.findContours(
            dilateOutput,
            contours,
            hierarchy,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        // Creates bounding rectangles along all of the detected contours
        val contoursPoly = arrayOfNulls<MatOfPoint2f>(contours.size)
        val boundRect: ArrayList<Rect> = arrayListOf()
        for (i in contours.indices) {
            contoursPoly[i] = MatOfPoint2f()
            Imgproc.approxPolyDP(
                MatOfPoint2f(*contours.get(i)!!.toArray()), contoursPoly[i], 3.0, true
            )
            boundRect[i] = Imgproc.boundingRect(MatOfPoint(*contoursPoly[i]!!.toArray()))
        }

        for (rect in boundRect) {
            Imgproc.rectangle(input, rect, VisionConstants.BOUNDING_RECTANGLE_COLOR)
            sampleDetections.add(SampleDetection(rect, color))
        }
    }

    @Synchronized
    fun getLargestDetection(color: SampleColor): SampleDetection? {
        if (sampleDetections.isEmpty) return null

        var largestDetection = SampleDetection(Rect(0, 0, 1, 1), color)

        for (detection in sampleDetections) {
            if (detection.color != color) continue
            if (detection.rect.area() <= largestDetection.rect.area()) continue

            largestDetection = detection
        }

        return largestDetection
    }
}