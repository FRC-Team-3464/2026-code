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
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class TrajectoryCalculator {

  private static final InterpolatingTreeMap<Double, TrajectoryParams> shooterTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), TrajectoryParams::interpolate);

  private static final double MIN_SHOOTING_DISTANCE = 1.5;
  private static final double MAX_SHOOTING_DISTANCE = 5.0;

  static {
    shooterTable.put(1.5, new TrajectoryParams(2800.0, 35.0, 0.38));
    shooterTable.put(2.0, new TrajectoryParams(3100.0, 38.0, 0.45));
    shooterTable.put(2.6289, new TrajectoryParams(5000.0, 42.0, 0.52));
    shooterTable.put(3.0, new TrajectoryParams(3650.0, 46.0, 0.60));
    shooterTable.put(3.5, new TrajectoryParams(3900.0, 50.0, 0.68));
    shooterTable.put(4.0, new TrajectoryParams(4100.0, 54.0, 0.76));
    shooterTable.put(4.5, new TrajectoryParams(4350.0, 58.0, 0.85));
    shooterTable.put(5.0, new TrajectoryParams(4550.0, 62.0, 0.94));
  }

  // ========== PUBLIC API ==========

  /**
   * Calculate shooter command for a single shooter. Use this when only one shooter needs
   * calculation.
   */
  public static ShooterCommand calculate(ShooterSide side, Translation2d targetLocation) {
    RobotStateData state = getCompensatedRobotState();
    return calculateWithState(side, targetLocation, state);
  }

  public static double calculateRPM(Translation2d targetLocation, Pose2d robotPose) {
    return shooterTable.get(targetLocation.getDistance(robotPose.getTranslation())).wheelRPM;
  }

  /**
   * Calculate shooter commands for both shooters efficiently. Use this when both shooters need
   * calculation - avoids duplicate state queries.
   */
  public static DualShooterCommands calculateBoth(Translation2d targetLocation) {
    RobotStateData state = getCompensatedRobotState();
    return new DualShooterCommands(
        calculateWithState(ShooterSide.LEFT, targetLocation, state),
        calculateWithState(ShooterSide.RIGHT, targetLocation, state));
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
      ShooterSide side, Translation2d targetLocation, RobotStateData state) {

    // 2. Identify Turret Offset and Position
    Transform3d robotToTurret;
    switch (side) {
      case LEFT:
        robotToTurret = TurretConstants.kRobotToLeftTurret;
        break;
      case RIGHT:
        robotToTurret = TurretConstants.kRobotToRightTurret;
      default:
        robotToTurret = new Transform3d();
        break;
    }

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

    Logger.recordOutput(
        "LaunchCalculator/" + side.getName() + "/LookaheadRobotPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/" + side.getName() + "/ShotVector",
        new Pose2d(lookaheadRobotPose.getTranslation(), turretAngleField));
    Logger.recordOutput("LaunchCalculator/" + side.getName() + "/Distance", lookaheadDistance);
    Logger.recordOutput(
        "LaunchCalculator/" + side.getName() + "/DistanceClamped", clampedFinalDistance);
    Logger.recordOutput(
        "LaunchCalculator/" + side.getName() + "/IsInRange",
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
