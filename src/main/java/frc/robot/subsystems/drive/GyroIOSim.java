package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import java.util.function.Supplier;

/** Simulates gyro heading from measured module travel, assuming wheels roll without slipping. */
public final class GyroIOSim implements GyroIO {
  private final Supplier<SwerveModulePosition[]> modulePositions;
  private final SwerveModulePosition[] previousPositions = {
    new SwerveModulePosition(), new SwerveModulePosition(),
    new SwerveModulePosition(), new SwerveModulePosition()
  };
  private Rotation2d yaw = Rotation2d.kZero;

  /**
   * Creates a simulated gyro for modules whose simulated travel starts at zero.
   *
   * @param modulePositions supplies refreshed positions in front-left, front-right, back-left,
   *     back-right order; evaluated during input updates, not construction
   */
  public GyroIOSim(Supplier<SwerveModulePosition[]> modulePositions) {
    this.modulePositions = modulePositions;
  }

  /** Advances yaw once from this cycle's measured wheel travel, then publishes gyro feedback. */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    SwerveModulePosition[] currentPositions = modulePositions.get();
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];
    for (int index = 0; index < deltas.length; index++) {
      deltas[index] =
          new SwerveModulePosition(
              currentPositions[index].distanceMeters - previousPositions[index].distanceMeters,
              currentPositions[index].angle);
      // Copy values so a supplier reusing mutable position objects cannot erase the next delta.
      previousPositions[index] =
          new SwerveModulePosition(
              currentPositions[index].distanceMeters, currentPositions[index].angle);
    }
    // Use actual model movement, not commanded speed: stopped or lagging wheels must not create
    // an idealized turn. Kinematics returns counterclockwise-positive rotation in radians.
    double deltaYaw = DriveConstants.kSwerveKinematics.toTwist2d(deltas).dtheta;
    yaw = yaw.plus(new Rotation2d(deltaYaw));
    inputs.connected = true;
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = deltaYaw / Constants.kLoopPeriodSeconds;
  }

  /** Sets the heading reference without resetting wheel history or discarding subsequent travel. */
  @Override
  public void setYaw(Rotation2d angle) {
    yaw = angle;
  }
}
