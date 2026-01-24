// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.KitBot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    SwerveSubsystem drivetrain = new SwerveSubsystem();
    KitBot kitbot = new KitBot();

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        LimelightHelpers.setPipelineIndex("", 1);
        drivetrain.setDefaultCommand(drivetrain.driveCommand(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY() * drivetrain.swerveDrive.getMaximumChassisVelocity(), 0.1),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX() * drivetrain.swerveDrive.getMaximumChassisVelocity(), 0.1),
            () -> LimelightHelpers.getTV("") && m_driverController.a().getAsBoolean() ? LimelightHelpers.getTX("") * 0.035 * drivetrain.swerveDrive.getMaximumChassisAngularVelocity() * -1.0 : MathUtil.applyDeadband(-m_driverController.getRightX() * drivetrain.swerveDrive.getMaximumChassisAngularVelocity(), 0.1),
            () -> false));
        m_driverController.a().whileTrue(kitbot.shoot());
        m_driverController.b().whileTrue(kitbot.intake());
        m_driverController.a().or(m_driverController.b()).onFalse(kitbot.stop());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
