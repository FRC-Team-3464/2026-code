package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  public default void updateInputs(TurretIOInputs inputs) {}

  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
  }

  public default void setPosition(Rotation2d position) {}

  /** Run turn motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  default void stop() {}
}
