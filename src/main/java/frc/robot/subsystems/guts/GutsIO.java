package frc.robot.subsystems.guts;

import org.littletonrobotics.junction.AutoLog;

public interface GutsIO {
  default void updateInputs(GutsIOInputs inputs) {
  }

  @AutoLog
  public static class GutsIOInputs {
    public double rightGutMotorVelocityRPM = 0.0;
    public double rightGutMotorPositionRot = 0.0;
    public double leftGutMotorVelocityRPM = 0.0;
    public double leftGutMotorPositionRot = 0.0;
  }

  default void setLeftGutMotorSpeed(double speed) {
  }

}
