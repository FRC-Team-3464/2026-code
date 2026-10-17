package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The RobotState class is the single source of truth for robot data. Any subsystem that needs to
 * access data outside itself will read from here.
 */
public class RobotState {
  // Makes one single RobotState object that holds all data (cannot be modified directly, must use
  // helper methods)
  private static RobotState instance = new RobotState();

  /** Returns the single RobotState instance. */
  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /** Pose Estimator */
  private SwerveDrivePoseEstimator poseEstimator;

  // Holds the robot's velocity, which is mainly used for shoot-on-the-move
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private RobotState() {
    // Initializes the pose estimator with a default pose
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
   * Update robot pose and robot-relative velocity from the same drive input refresh. Requiring both
   * in one observation prevents a caller from updating pose while forgetting to publish speed.
   *
   * @param observation An {@link OdometryObservation} object representing the measured odometry
   *     state.
   */
  public void addOdometryObservation(OdometryObservation observation) {

    // if (observation.gyroAngle().isEmpty()) {
    // // Don't update pose without gyro
    // return;
    // }

    // Sends an updated drivetrain pose to the PoseEstimator object
    poseEstimator.updateWithTime(
        observation.timestamp(), observation.gyroAngle(), observation.modulePositions());
    robotVelocity = observation.robotRelativeSpeeds();

    //  Log updated estimated poses with AdvantageKit
    Logger.recordOutput("RobotState/EstimatedPose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/EstimatedRotation",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  /**
   * Update robot pose estimate from cameras.
   *
   * @param measurement A {@link VisionMeasurement} object representing the vision pose estimate.
   */
  public void addVisionMeasurement(VisionMeasurement measurement) {
    // Sends a vision-only measurement to the PoseEstimator object
    poseEstimator.addVisionMeasurement(
        measurement.visionPose(), measurement.timestamp(), measurement.stdDevs());

    // Log updated estimated poses with AdvantageKit
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
    poseEstimator.resetPose(pose);
  }

  public void resetRotation(Rotation2d rotation) {
    poseEstimator.resetRotation(rotation);
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
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /** Get the current robot velocity relative to the field. */
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /** Returns the alliance hub. Autonomous shooting always uses this target. */
  public Translation2d getAutoShooterTarget() {
    return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
  }

  public enum ShotType {
    HUB,
    PASS,
    UNAVAILABLE
  }

  /** A hub or home-side passing target in field coordinates. */
  public record ShooterTarget(ShotType type, Translation2d position) {}

  /** Selects the hub in our alliance zone, otherwise the nearest home-side passing point. */
  public ShooterTarget selectTeleopShooterTarget() {
    Translation2d position = getEstimatedPose().getTranslation();
    if (DriverStation.getAlliance().isEmpty()
        || !Double.isFinite(position.getX())
        || !Double.isFinite(position.getY())
        || !Double.isFinite(getEstimatedPose().getRotation().getRadians())) {
      return new ShooterTarget(ShotType.UNAVAILABLE, new Translation2d());
    }
    Translation2d relative = AllianceFlipUtil.apply(position);
    if (relative.getX() <= FieldConstants.LinesVertical.allianceZone) {
      return new ShooterTarget(ShotType.HUB, getAutoShooterTarget());
    }
    // Proposed passing destinations, measured from the blue origin and mirrored for red.
    double targetY =
        relative.getY() > FieldConstants.LinesHorizontal.center
            ? FieldConstants.fieldWidth - 1.0
            : 1.0;
    return new ShooterTarget(
        ShotType.PASS, AllianceFlipUtil.apply(new Translation2d(2.0, targetY)));
  }

  // Helper records (objects that just store data) to standardize sharing data between classes
  public record OdometryObservation(
      double timestamp,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroAngle,
      ChassisSpeeds robotRelativeSpeeds) {}

  public record VisionMeasurement(double timestamp, Pose2d visionPose, Matrix<N3, N1> stdDevs) {}
}
