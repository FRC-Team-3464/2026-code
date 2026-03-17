package frc.robot.subsystems.guts;

import org.littletonrobotics.junction.AutoLog;

/**
 * This IO interface contains the class which initializes all the inputs as well as default methods
 * to update the values of the inputs and set the speed of the motor.
 *
 * @author Ryan Hefferon
 */
public interface GutsIO {

  /** Updates the values of all the inputs using the physical encoders. */
  default void updateInputs(GutsIOInputs inputs) {}

  /** Contains all the inputs regarding motors to be stored as data. */
  @AutoLog
  public static class GutsIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentDrawAmps = 0.0;
  }

  /** Sets the gut motor to a specific speed ranging from -1.0 to 1.0 */
  default void setOpenLoop(double speed) {}
}
