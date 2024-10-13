package org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.mecanumdrive.MecanumDriveSubsystem;
import java.util.function.DoubleSupplier;

public final class FieldCentricCommand extends CommandBase {
    @NonNull
    private final MecanumDriveSubsystem mecanumDriveSubsystem;

    @NonNull
    private final DoubleSupplier driveSupplier,
                                 strafeSupplier,
                                 turnSupplier;

    public FieldCentricCommand(
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
        mecanumDriveSubsystem.fieldCentric(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );
    }
}
