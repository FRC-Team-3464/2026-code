package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  public static class HoodIOInputs {
    boolean connected = false;
    double angleRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
  }

  default void setAngle(double angle) {}
}
