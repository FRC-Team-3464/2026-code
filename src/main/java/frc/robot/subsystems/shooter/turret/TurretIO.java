package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  public default void updateInputs(TurretIOInputs inputs) {}

  @AutoLog
  public static class TurretIOInputs {
    boolean connected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
  }

  public default void setPosition(Rotation2d position) {}
}
