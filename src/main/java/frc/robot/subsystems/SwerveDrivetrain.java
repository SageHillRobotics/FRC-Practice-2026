package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveDrivetrain extends SubsystemBase {
    SwerveDrive swerveDrive;

    public SwerveDrivetrain() {
        try {
            // SDS MK4i L1 NEO
            this.swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(12.5));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Command driveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier fieldRelativeSupplier) {
        return Commands.run(() -> swerveDrive.drive(
            new Translation2d(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble()), 
            rotationSupplier.getAsDouble(),
            fieldRelativeSupplier.getAsBoolean(),
            false
        ), this);
    }

    public Command driveJoystickCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier fieldRelativeSupplier) {
        return driveCommand(
            () -> translationXSupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
            () -> translationYSupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
            () -> rotationSupplier.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            fieldRelativeSupplier
        );
    }
}
