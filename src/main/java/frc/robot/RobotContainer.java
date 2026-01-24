// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.YAGSLSwerveSubsystem;
import frc.robot.subsystems.KitBot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    YAGSLSwerveSubsystem drivetrain = new YAGSLSwerveSubsystem();
    KitBot kitbot = new KitBot();

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.driveCommand(() -> MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1) * YAGSLSwerveSubsystem.maximumSpeed, () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1) * YAGSLSwerveSubsystem.maximumSpeed, () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1) * Math.PI / 2, () -> false));
        m_driverController.a().whileTrue(kitbot.shoot());
        m_driverController.b().whileTrue(kitbot.intake());
        m_driverController.x().whileTrue(kitbot.eject());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
