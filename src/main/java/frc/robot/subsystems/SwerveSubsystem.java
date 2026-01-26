package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    // TODO!: adjust gear ratio in maximum speed calculation and in swerve config files
    public static double maximumSpeed = 5676 / 6.75 * Math.PI * Units.inchesToMeters(4) / 60;
    
    public SwerveDrive swerveDrive;

    private PIDController autoXController = new PIDController(10.0, 0.0, 0.0);
    private PIDController autoYController = new PIDController(10.0, 0.0, 0.0);
    private PIDController autoHeadingController = new PIDController(7.0, 0.0, 0.0);
    
    public void followTrajectory(SwerveSample sample) {
        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d pose = swerveDrive.getPose();

        ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += autoXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += autoYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += autoHeadingController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        swerveDrive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, pose.getRotation()));
    }

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("X: " + swerveDrive.getPose().getX() + " Y: " + swerveDrive.getPose().getY() + " Rot: " + swerveDrive.getPose().getRotation().getDegrees());
    }

    public Command driveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier headingSupplier, BooleanSupplier fieldOriented) {
        return Commands.run(() -> swerveDrive.drive(new Translation2d(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble()),
                        headingSupplier.getAsDouble(),
                        fieldOriented.getAsBoolean(),
                        false), this);
    }
}
