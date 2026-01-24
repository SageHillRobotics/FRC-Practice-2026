package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitBot extends SubsystemBase {
    private SparkMax motor1;
    private SparkMax motor2;

    public KitBot() {
        motor1 = new SparkMax(14, SparkMax.MotorType.kBrushed);
        motor2 = new SparkMax(15, SparkMax.MotorType.kBrushed);
    }

    public Command intake() {
        return Commands.runOnce(() -> {
            motor1.set(-1);
            motor2.set(-1);
        });
    }

    public Command shoot() {
        return Commands.runOnce(() -> {
            motor1.set(-1);
            motor2.set(1);
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            motor1.set(0);
            motor2.set(0);
        });
    }
}
