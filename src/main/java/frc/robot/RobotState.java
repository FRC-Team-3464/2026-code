package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance = new RobotState();

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /** Pose Estimator */
  private SwerveDrivePoseEstimator poseEstimator;

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private Rotation2d gyroOffset = new Rotation2d();

  private RobotState() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kSwerveKinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            Pose2d.kZero);
  }

  /**
   * Update robot pose estimate from drive sensors.
   *
   * @param observation An {@link OdometryObservation} object representing the measured odometry
   *     state.
   */
  public void addOdometryObservation(OdometryObservation observation) {

    // if (observation.gyroAngle().isEmpty()) {
    // // Don't update pose wihtout gyro
    // return;
    // }

    poseEstimator.updateWithTime(
        observation.timestamp(), observation.gyroAngle(), observation.modulePositions());

    Logger.recordOutput("RobotState/EstimatedPose", poseEstimator.getEstimatedPosition());
  }

  /**
   * Update robot pose estimate from cameras.
   *
   * @param measurement A {@link VisionMeasurement} object representing the vision pose estimate.
   */
  public void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.visionPose().toPose2d(), measurement.timestamp(), measurement.stdDevs());

    Logger.recordOutput("RobotState/EstimatedPose", poseEstimator.getEstimatedPosition());
  }

  /**
   * Reset pose estimate and align gyro frame to the given pose.
   *
   * @param pose The pose to reset the pose estimator to.
   * @param modulePositions An array of the current swerve module positions.
   * @param rawGyroRotation The estimated rotation from the drivetrain.
   */
  public void setPose(
      Pose2d pose, SwerveModulePosition[] modulePositions, Rotation2d rawGyroRotation) {
    poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);
  }

  /**
   * Reset pose estimate and align gyro frame to the given pose.
   *
   * @param pose The pose to reset the pose estimator to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation(), null, pose);
  }

  public void resetRotation(Rotation2d rotation) {
    gyroOffset = poseEstimator.getEstimatedPosition().getRotation().minus(rotation);
  }

  /**
   * Set the robot's velocity.
   *
   * @param speeds A ChassisSpeeds object representing the robot's current velocity
   */
  public void setRobotVelocity(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  /**
   * Field-relative estimated robot pose.
   *
   * @return A Pose2d object representing the robot's estimated pose.
   */
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Current robot velocity.
   *
   * @return A ChassisSpeeds object representing the velocity of the robot.
   */
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation().minus(gyroOffset);
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  public record OdometryObservation(
      double timestamp, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {}

  public record VisionMeasurement(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}
}
