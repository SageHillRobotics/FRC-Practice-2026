package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;
    // TODO!: adjust gear ratio in maximum speed calculation and in swerve config files
    public static double maximumSpeed = 5676 / 6.75 * Math.PI * Units.inchesToMeters(4) / 60;
    public SwerveSubsystem() {
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            throw new RuntimeException("Failed to initialize swerve drive", e);
        }
    }

    // public Command driveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier headingXSupplier, DoubleSupplier headingYSupplier) {
    //     return new Command() {
    //         @Override
    //         public void initialize() {
    //             swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), headingXSupplier.getAsDouble(), headingYSupplier.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), maximumSpeed));
    //         }
    //     };
    // }

    public Command driveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier headingSupplier, BooleanSupplier fieldOriented) {
        return new Command() {
            @Override
            public void execute() {
                swerveDrive.drive(new Translation2d(translationXSupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), translationYSupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), headingSupplier.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), fieldOriented.getAsBoolean(), false);
            }
        };
    }
}
