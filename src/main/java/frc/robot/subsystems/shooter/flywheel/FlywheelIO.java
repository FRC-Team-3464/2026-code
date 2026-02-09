package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  default void updateInputs(FlywheelIOInputs inputs) {}

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
  }

  /**
   * Set the shooter motor to a specified velocity.
   *
   * @param velocity The velocity to set the motor to (in RPM).
   */
  default void setVelocity(double velocity) {}

  default void stop() {}
}
