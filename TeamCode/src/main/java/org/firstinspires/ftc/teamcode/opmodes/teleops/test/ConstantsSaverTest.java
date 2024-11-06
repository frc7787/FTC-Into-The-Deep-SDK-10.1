package org.firstinspires.ftc.teamcode.opmodes.teleops.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ConstantsSaver;

@TeleOp(name = "Test - Constants Saver")
@Disabled
public final class ConstantsSaverTest extends CommandOpMode {

    @Override public void initialize() {
        new ConstantsSaver(telemetry).save();
    }
}
