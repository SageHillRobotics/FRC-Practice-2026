package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(28);
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(28);

    private SwerveModule[] swerveModules = new SwerveModule[]{
        new SwerveModule(1, 2, true, true),
        new SwerveModule(3, 4, true, true),
        new SwerveModule(5, 6, true, true),
        new SwerveModule(7, 8, true, true)
    };

    private Pigeon2 pigeon = new Pigeon2(13);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2),
        new Translation2d(CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2),
        new Translation2d(-CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2),
        new Translation2d(-CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2)
    );

    public Command drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotationRate) {
        return run(() -> {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotationRate.getAsDouble(),
                Rotation2d.fromRadians(getHeading())
            ));
            setModuleStates(states);
        });
    }

    public double getHeading() {
        return -MathUtil.angleModulus(Math.toRadians(pigeon.getYaw().getValueAsDouble()));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.MAX_SPEED);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }
}