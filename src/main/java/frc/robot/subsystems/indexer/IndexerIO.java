package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean throatConnected = false;
    public double throatVelocityRadPerSec = 0.0;
    public double throatAppliedVolts = 0.0;
    public double throatCurrentDrawAmps = 0.0;

    public boolean toungeConnected = false;
    public double toungeVelocityRadPerSec = 0.0;
    public double toungeAppliedVolts = 0.0;
    public double toungeCurrentDrawAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setThroatOpenLoop(double output) {}

  default void setToungeOpenLoop(double output) {}

  default void stop() {}
}
