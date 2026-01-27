// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.KitBot;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final SwerveSubsystem drivetrain = new SwerveSubsystem();
    private final KitBot kitbot = new KitBot();

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final AutoFactory autoFactory = new AutoFactory(
            drivetrain.swerveDrive::getPose,
            drivetrain.swerveDrive::resetOdometry,
            drivetrain::followTrajectory,
            true,
            drivetrain);;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY() * drivetrain.swerveDrive.getMaximumChassisVelocity(), 0.2),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX() * drivetrain.swerveDrive.getMaximumChassisVelocity(), 0.2),
            () -> aimAssistCorrectId() && m_driverController.a().getAsBoolean() ? LimelightHelpers.getTX("") * 0.01 * drivetrain.swerveDrive.getMaximumChassisAngularVelocity() * -1.0 : MathUtil.applyDeadband(-m_driverController.getRightX() * drivetrain.swerveDrive.getMaximumChassisAngularVelocity(), 0.2),
            () -> false));
        kitbot.setDefaultCommand(kitbot.idle());
        m_driverController.a().whileTrue(kitbot.shoot());
        m_driverController.b().whileTrue(kitbot.intake());
    }

    public Command getAutonomousCommand() {
        return autoFactory.trajectoryCmd("5m_forward_180");
    }

    public boolean aimAssistCorrectId() {
        return LimelightHelpers.getFiducialID("") == 9 || LimelightHelpers.getFiducialID("") == 10 || LimelightHelpers.getFiducialID("") == 25 || LimelightHelpers.getFiducialID("") == 26;
    }
}
