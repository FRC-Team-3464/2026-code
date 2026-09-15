package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean throatConnected = false;
    public double throatVelocityRadPerSec = 0.0;
    public double throatAppliedVolts = 0.0;
    public double throatCurrentDrawAmps = 0.0;

    public boolean tongueConnected = false;
    public double tongueVelocityRadPerSec = 0.0;
    public double tongueAppliedVolts = 0.0;
    public double tongueCurrentDrawAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  /**
   * Sets the throat motor to a specific speed ranging from -1.0 to 1.0. -1.0 represents 100%
   * maximum reverse speed, while 1.0 represents 100% forward speed.
   */
  default void setThroatOpenLoop(double output) {}

  /**
   * Sets the tongue motor to a specific speed ranging from -1.0 to 1.0. -1.0 represents 100%
   * maximum reverse speed, while 1.0 represents 100% forward speed.
   */
  default void setTongueOpenLoop(double output) {}

  /** Stops all indexer motors. */
  default void stop() {}
}
