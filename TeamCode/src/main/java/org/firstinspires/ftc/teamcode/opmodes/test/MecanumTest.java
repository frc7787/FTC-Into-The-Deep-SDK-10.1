package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.utility.playstationcontroller.PlayStationController;

@TeleOp(name = "Test - Mecanum")
public final class MecanumTest extends CommandOpMode {

    @Override public void initialize() {
        MecanumDriveSubsystem mecanumDriveSubsystem = new MecanumDriveSubsystem(this);
        PlayStationController driverController      = new PlayStationController(gamepad1);

        register(mecanumDriveSubsystem);

        schedule(
                new RobotCentricCommand(
                        mecanumDriveSubsystem,
                        () -> driverController.getLeftY() * Math.abs(driverController.getLeftY()),
                        () -> (driverController.getLeftX() * Math.abs(driverController.getLeftX())),
                        () -> driverController.getRightX() * Math.abs(driverController.getRightX())
                )
        );
    }
}
