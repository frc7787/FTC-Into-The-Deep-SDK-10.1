package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public final class SetIntakePositionAction implements Action {
    private final Arm arm;
    private final double position;
    private final double timeOutMS;
    private final ElapsedTime timer;

    private boolean initialized;

    public SetIntakePositionAction(@NonNull Arm arm, double position, double timeOutMS) {
        this.arm = arm;
        this.position = position;
        this.timeOutMS = timeOutMS;
        initialized = false;
        timer = new ElapsedTime();
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            arm.setIntakePosition(position);
            timer.reset();
            initialized = true;
        }

        return timer.milliseconds() < timeOutMS;
    }
}
