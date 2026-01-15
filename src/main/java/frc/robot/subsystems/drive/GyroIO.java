package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The {@code GyroIO} interface defines methods and attributes for the gyro. */
public interface GyroIO {
  default void updateInputs(GyroIOInputs inputs) {}
  ;

  @AutoLog
  /** Gyro values */
  public static class GyroIOInputs {
    public GyroIOData data = new GyroIOData(false, Rotation2d.kZero, 0);
  }

  public record GyroIOData(
      boolean connected, Rotation2d yawPosition, double yawVelocityRadPerSec) {}
  ;

  /** Resets the gyro. */
  default void resetGyro() {}
  ;

  /** Zeroes the yaw. */
  default void zeroYaw() {}
  ;
}
