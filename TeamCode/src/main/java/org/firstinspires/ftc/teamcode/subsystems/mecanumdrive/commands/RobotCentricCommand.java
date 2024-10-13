package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import java.util.function.DoubleSupplier;

public final class RobotCentricCommand extends CommandBase {
        @NonNull
        private final MecanumDriveSubsystem mecanumDriveSubsystem;

        @NonNull
        private final DoubleSupplier driveSupplier, strafeSupplier, turnSupplier;

        public RobotCentricCommand(
                @NonNull MecanumDriveSubsystem mecanumDriveSubsystem,
                @NonNull DoubleSupplier driveSupplier,
                @NonNull DoubleSupplier strafeSupplier,
                @NonNull DoubleSupplier turnSupplier
        ) {
            this.mecanumDriveSubsystem = mecanumDriveSubsystem;
            this.driveSupplier         = driveSupplier;
            this.strafeSupplier        = strafeSupplier;
            this.turnSupplier          = turnSupplier;

            addRequirements(mecanumDriveSubsystem);
        }

        @Override public void execute() {
            mecanumDriveSubsystem.robotCentric(
                    driveSupplier.getAsDouble(),
                    strafeSupplier.getAsDouble(),
                    turnSupplier.getAsDouble()
            );
        }
}
