package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code IntakeIO} class provides methods for interacting with the intake motors and updating
 * the intake inputs.
 *
 * @author Ryan Hefferon
 * @author Matthew McGrath
 * @author Maxwell Morgan
 * @author Julien Precourt
 */
public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  @AutoLog
  public static class IntakeIOInputs {
    public double pivotVelocityRadPerSec = 0.0;
    public double wheelVelocityRadPerSec = 0.0;

    public double pivotPositionRad = 0.0;
    public double wheelPositionRad = 0.0;

    public double pivotAppliedVolts = 0.0;
    public double wheelAppliedVolts = 0.0;
    
    public double pivotCurrentDrawAmps = 0.0;
    public double wheelCurrentDrawAmps = 0.0;
  }

  default void setPivotPosition(double positionRad) {}

  /**
   * method to set the speed of the pivot
   *
   * @param speed determines the speed of the pivot on a scale of -1 to 1
   */
  default void setPivotSpeed(double speed) {}

  /**
   * method to set the speed of the wheel
   *
   * @param speed determines the speed of the wheel on a scale of -1 to 1
   */
  default void setWheelSpeed(double speed) {}
}
