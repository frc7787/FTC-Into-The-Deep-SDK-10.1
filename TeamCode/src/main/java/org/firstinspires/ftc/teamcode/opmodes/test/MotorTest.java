package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "Test - Motor", group = "Test")
public final class MotorTest extends OpMode {
    DcMotorImplEx motorOne, motorTwo;

    @Override public void init() {
       motorOne = hardwareMap.get(DcMotorImplEx.class, "motorOne");
       motorTwo = hardwareMap.get(DcMotorImplEx.class, "motorTwo");
    }

    @Override public void loop() {
        motorOne.setPower(1.0);
        motorTwo.setPower(1.0);
    }
}
