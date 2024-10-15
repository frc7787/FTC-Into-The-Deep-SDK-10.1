package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public final class MotorGroup {
    DcMotor[] motors;

    public MotorGroup(@NonNull Direction direction, @NonNull DcMotor ... motors) {
       this.motors = motors;

       for (DcMotor motor : motors) {
           motor.setDirection(direction);
       }
    }

    public void setDirection(@NonNull Direction direction) {
        for (DcMotor motor : motors) {
            motor.setDirection(direction);
        }
    }

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }
}
