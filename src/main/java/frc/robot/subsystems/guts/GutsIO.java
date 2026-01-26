package frc.robot.subsystems.guts;

import org.littletonrobotics.junction.AutoLog;

public interface GutsIO {
  default void updateInputs(GutsIOInputs inputs) {}

  @AutoLog
  public static class GutsIOInputs {}
}
