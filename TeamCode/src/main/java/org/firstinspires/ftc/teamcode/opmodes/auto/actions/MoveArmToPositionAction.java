package org.firstinspires.ftc.teamcode.opmodes.auto.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmDebug;

public final class MoveArmToPositionAction implements Action {
    private final Arm arm;
    private ArmDebug armDebug;
    private final double horizontalInches, verticalInches;
    private boolean initialized;
    private ElapsedTime timer;

    public MoveArmToPositionAction(
            @NonNull Arm arm,
            double verticalInches,
            double horizontalInches
    ) {
        this.arm = arm;
        this.verticalInches = verticalInches;
        this.horizontalInches = horizontalInches;
        initialized = false;
        timer = new ElapsedTime();
    }

    @Override public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("X", arm.horizontalInchesRobotCentric());
        telemetryPacket.put("Y", arm.verticalInchesRobotCentric());

        if (!initialized) {
            arm.setTargetInchesRobotCentric(horizontalInches, verticalInches);
            timer.reset();
            initialized = true;
        }
        arm.update();

        boolean isFinished = !arm.isAtPosition() || timer.seconds() > 3.0;

        if (isFinished) arm.stop();

        return isFinished;
    }
}
