package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /** Pose Estimator */
  private SwerveDrivePoseEstimator poseEstimator;

  private RobotState() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.swerveKinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            Pose2d.kZero);
  }

  /** Update robot state from drive sensors. */
  public void addOdometryObservation(OdometryObservation observation) {

    // if (observation.gyroAngle().isEmpty()) {
    // // Don't update pose wihtout gyro
    // return;
    // }

    poseEstimator.updateWithTime(
        observation.timestamp(), observation.gyroAngle(), observation.modulePositions());

    Logger.recordOutput("RobotState/EstimatedPose", poseEstimator.getEstimatedPosition());
  }

  public void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.visionPose().toPose2d(), measurement.timestamp(), measurement.stdDevs());

    Logger.recordOutput("RobotState/EstimatedPose", poseEstimator.getEstimatedPosition());
  }

  /** Reset pose estimate and align gyro frame to the given pose. */
  public void setPose(
      Pose2d pose, SwerveModulePosition[] modulePositions, Rotation2d rawGyroRotation) {
    poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);
  }

  /** Field-relative estimated robot pose. */
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public record OdometryObservation(
      double timestamp, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {}

  public record VisionMeasurement(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}
}
