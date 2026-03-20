package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class TrajectoryCalculator {

  private static final InterpolatingTreeMap<Double, TrajectoryParams> shooterTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), TrajectoryParams::interpolate);

  private static final double MIN_SHOOTING_DISTANCE = 1.5;
  private static final double MAX_SHOOTING_DISTANCE = 5.0;

  static {
    shooterTable.put(2.36, new TrajectoryParams(2000.0, 0, 0.45));
    shooterTable.put(2.6, new TrajectoryParams(2100.0, -0.5, 0.52));
    shooterTable.put(3.0, new TrajectoryParams(2150.0, -1.0, 0.60));
    shooterTable.put(3.5, new TrajectoryParams(2350.0, -1.5, 0.68));
    shooterTable.put(4.0, new TrajectoryParams(2500.0, -1.75, 0.76));
    shooterTable.put(4.5, new TrajectoryParams(3000.0, -2, 0.76));
  }

  // ========== PUBLIC API ==========

  /**
   * Calculate shooter command for a single shooter. Use this when only one shooter needs
   * calculation.
   */
  public static ShooterCommand calculate(Translation2d targetLocation) {
    RobotStateData state = getCompensatedRobotState();
    return calculateWithState(targetLocation, state);
  }

  public static double calculateRPM(Translation2d targetLocation, Pose2d robotPose) {
    return shooterTable.get(targetLocation.getDistance(robotPose.getTranslation())).wheelRPM;
  }

  public static double calculateHoodAngle(Translation2d targetLocation, Pose2d robotPose) {
    return shooterTable.get(targetLocation.getDistance(robotPose.getTranslation())).hoodAngle;
  }

  // ========== PRIVATE IMPLEMENTATION ==========

  /** Get and compensate robot state (shared between both shooters). */
  private static RobotStateData getCompensatedRobotState() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVel = RobotState.getInstance().getRobotVelocity();
    ChassisSpeeds fieldVel = RobotState.getInstance().getFieldVelocity();

    Pose2d compensatedRobotPose =
        robotPose.exp(
            new Twist2d(
                robotRelativeVel.vxMetersPerSecond * ShooterConstants.kLatencySeconds,
                robotRelativeVel.vyMetersPerSecond * ShooterConstants.kLatencySeconds,
                robotRelativeVel.omegaRadiansPerSecond * ShooterConstants.kLatencySeconds));

    return new RobotStateData(compensatedRobotPose, robotRelativeVel, fieldVel);
  }

  /** Calculate shooter command for a specific side using pre-computed robot state. */
  private static ShooterCommand calculateWithState(
      Translation2d targetLocation, RobotStateData state) {

    // 2. Identify Turret Offset and Position
    Transform3d robotToTurret = TurretConstants.kRobotToTurret;

    Pose2d turretPose =
        state.compensatedRobotPose.transformBy(GeomUtil.toTransform2d(robotToTurret));

    // 3. Calculate Field-Relative Turret Velocity (Linear + Tangential)
    Translation2d tangentialVelRobot =
        new Translation2d(
            -state.robotRelativeVel.omegaRadiansPerSecond * robotToTurret.getY(),
            state.robotRelativeVel.omegaRadiansPerSecond * robotToTurret.getX());

    Translation2d totalTurretVel =
        new Translation2d(state.fieldVel.vxMetersPerSecond, state.fieldVel.vyMetersPerSecond)
            .plus(tangentialVelRobot.rotateBy(state.compensatedRobotPose.getRotation()));

    // 4. Iterative Lookahead
    double lookaheadDistance = targetLocation.getDistance(turretPose.getTranslation());
    Translation2d predictedTurretTranslation = turretPose.getTranslation();

    for (int i = 0; i < 10; i++) {
      double clampedDistance =
          Math.max(MIN_SHOOTING_DISTANCE, Math.min(MAX_SHOOTING_DISTANCE, lookaheadDistance));
      double timeOfFlight = shooterTable.get(clampedDistance).timeOfFlight();
      predictedTurretTranslation =
          turretPose.getTranslation().plus(totalTurretVel.times(timeOfFlight));
      lookaheadDistance = targetLocation.getDistance(predictedTurretTranslation);
    }

    // 5. Final Angles and Parameters
    Rotation2d turretAngleField = targetLocation.minus(predictedTurretTranslation).getAngle();
    Rotation2d turretAngleRobot = turretAngleField.minus(state.compensatedRobotPose.getRotation());

    double clampedFinalDistance =
        Math.max(MIN_SHOOTING_DISTANCE, Math.min(MAX_SHOOTING_DISTANCE, lookaheadDistance));
    TrajectoryParams params = shooterTable.get(clampedFinalDistance);

    // 6. AdvantageScope Logging
    Pose2d lookaheadTurretPose =
        new Pose2d(predictedTurretTranslation, state.compensatedRobotPose.getRotation());
    Pose2d lookaheadRobotPose =
        lookaheadTurretPose.transformBy(GeomUtil.toTransform2d(robotToTurret).inverse());

    Logger.recordOutput("LaunchCalculator/LookaheadRobotPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/ShotVector",
        new Pose2d(lookaheadRobotPose.getTranslation(), turretAngleField));
    Logger.recordOutput("LaunchCalculator/Distance", lookaheadDistance);
    Logger.recordOutput("LaunchCalculator/DistanceClamped", clampedFinalDistance);
    Logger.recordOutput(
        "LaunchCalculator/IsInRange",
        lookaheadDistance >= MIN_SHOOTING_DISTANCE && lookaheadDistance <= MAX_SHOOTING_DISTANCE);

    return new ShooterCommand(params.wheelRPM(), params.hoodAngle(), turretAngleRobot);
  }

  // Helper for debug if needed
  public double getHorizontalVelocity(double distance) {
    double clampedDistance =
        Math.max(MIN_SHOOTING_DISTANCE, Math.min(MAX_SHOOTING_DISTANCE, distance));
    TrajectoryParams params = shooterTable.get(clampedDistance);
    return clampedDistance / params.timeOfFlight();
  }

  // ========== DATA RECORDS ==========

  /** Robot state data that's shared between both shooters */
  private record RobotStateData(
      Pose2d compensatedRobotPose, ChassisSpeeds robotRelativeVel, ChassisSpeeds fieldVel) {}

  /** Shooter commands for both shooters */
  public record DualShooterCommands(ShooterCommand left, ShooterCommand right) {}

  public record TrajectoryParams(double wheelRPM, double hoodAngle, double timeOfFlight)
      implements Interpolatable<TrajectoryParams> {
    @Override
    public TrajectoryParams interpolate(TrajectoryParams other, double t) {
      return new TrajectoryParams(
          wheelRPM + (other.wheelRPM - wheelRPM) * t,
          hoodAngle + (other.hoodAngle - hoodAngle) * t,
          timeOfFlight + (other.timeOfFlight - timeOfFlight) * t);
    }
  }

  public record ShooterCommand(double wheelRPM, double hoodAngle, Rotation2d turretAngle) {}
}
