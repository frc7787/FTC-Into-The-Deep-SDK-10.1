package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public final class MotorGroup {
    DcMotorImplEx[] motors;

    public MotorGroup(@NonNull Direction direction, @NonNull DcMotorImplEx ... motors) {
       this.motors = motors;

       for (DcMotorImplEx motor : motors) {
           motor.setDirection(direction);
       }
    }

    public void setDirection(@NonNull Direction direction) {
        for (DcMotorImplEx motor : motors) {
            motor.setDirection(direction);
        }
    }

    public void setPower(double power) {
        for (DcMotorImplEx motor : motors) {
            motor.setPower(power);
        }
    }
}
