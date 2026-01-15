package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code ModuleIO} class contains default methods for the
 *
 * @author Maxwell Morgan
 */
public interface ModuleIO {
  default void updateInputs(ModuleIOInputs inputs) {}

  @AutoLog
  /** Module values */
  public class ModuleIOInputs {
    public ModuleIOData data =
        new ModuleIOData(false, 0, 0, 0, false, Rotation2d.kZero, 0, 0, 0, 0);
  }

  public record ModuleIOData(
      boolean driveConnected,
      double drivePositionRad,
      double driveVelocityRadPerSec,
      double driveAppliedVolts,
      boolean turnConnected,
      Rotation2d turnPosition,
      double turnVelocityRadPerSec,
      double turnAppliedVolts,
      double driveCurrentAmps,
      double turnCurrentAmps) {}

  /**
   * Sets the drive motor output.
   *
   * @param percentOutput the percent of the drive motor's maximum output to request (between -1 and
   *     1)
   */
  default void runDriveDutyCycle(double percentOutput) {}

  /**
   * Sets the turn motor output.
   *
   * @param percentOutput the percent of the turn motor's maximum output to request (between -1 and
   *     1)
   */
  default void runTurnDutyCycle(double percentOutput) {}

  /**
   * Sets the drive motor velocity.
   *
   * @param velocityRadPerSec the velocity used to set the drive motor controller target
   */
  default void runDriveVelocity(double velocityRadPerSec) {}

  /**
   * Sets the turn motor to the specified angle.
   *
   * @param angle the {@link Rotation2d} used to set the angle motor controller target
   */
  default void runTurnAngle(Rotation2d angle) {}

  /** Resets the turning encoder to match absolute CANcoder. */
  default void resetToAbsolute() {}
}
