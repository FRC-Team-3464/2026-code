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
    public boolean leftPivotConnected = false;
    public double leftPivotVelocityRadPerSec = 0.0;
    public double leftPivotAppliedVolts = 0.0;
    public double leftPivotCurrentDrawAmps = 0.0;

    public boolean rightPivotConnected = false;
    public double rightPivotVelocityRadPerSec = 0.0;
    public double rightPivotAppliedVolts = 0.0;
    public double rightPivotCurrentDrawAmps = 0.0;

    public boolean driveConnected = false;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentDrawAmps = 0.0;
  }

  default void setPivotPosition(double positionRotations) {}

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
