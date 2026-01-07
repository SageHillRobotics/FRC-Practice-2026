package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem {
    public static double maximumSpeed = Units.feetToMeters(4.5);
    private SwerveDrive swerveDrive;

    public SwerveSubsystem() throws IOException {
        swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(maximumSpeed);
    }

    public Command driveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier headingXSupplier, DoubleSupplier headingYSupplier) {
        return new Command() {
            @Override
            public void initialize() {
                swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationXSupplier.getAsDouble() * 0.8, translationYSupplier.getAsDouble() * 0.8, headingXSupplier.getAsDouble(), headingYSupplier.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), maximumSpeed));
            }
        };
    }
}
