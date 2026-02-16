package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
  }

  /**
   * Sets the target angle for the hood.
   *
   * @param angle The angle for the hood to aim at (in radians).
   */
  default void setAngle(double angle) {}

  /** Run turn motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  default void stop() {}
}
