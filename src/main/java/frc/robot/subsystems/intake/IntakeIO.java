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
  /**
   * method to set the speed of the arm
   * @param speed determines the speed of the arm on a scale of -1 to 1
   */
  default void setArmSpeed(double speed){}
    /**
   * method to set the speed of the wheel
   * @param speed determines the speed of the wheel on a scale of -1 to 1
   */
  default void setWheelSpeed(double speed){}
}
