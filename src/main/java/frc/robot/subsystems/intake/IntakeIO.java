package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code IntakeIO} class provides methods for interacting with the intake
 * motors and updating
 * the intake inputs.
 *
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Maxwell Morgan
 * @author Julien Precourt
 */
public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {
  }

  @AutoLog
  public static class IntakeIOInputs {
    double armMotorVelocityRPM = 0.0;
    double wheelMotorVelocityRPM = 0.0;
    double armMotorPositionsRotations = 0.0;
    double wheelMotorPositionRotations = 0.0;

  }

  default void setArmSpeed(double speed){}
  default void setWheelSpeed(double speed){}
}
