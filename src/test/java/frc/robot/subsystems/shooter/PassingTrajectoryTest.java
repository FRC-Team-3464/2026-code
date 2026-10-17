package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import org.junit.jupiter.api.Test;

class PassingTrajectoryTest {
  @Test
  void passingInterpolatesFromTurretDistanceAndClampsWithoutChangingHubTable() {
    Pose2d pose = Pose2d.kZero;
    Translation2d turret =
        pose.transformBy(GeomUtil.toTransform2d(TurretConstants.kRobotToTurret)).getTranslation();
    var middle = TrajectoryCalculator.calculatePassing(turret.plus(new Translation2d(5, 0)), pose);
    assertEquals(1925, middle.wheelRPM(), 1e-9);
    assertEquals(-0.5, middle.hoodAngle(), 1e-9);
    assertEquals((1.11 + 1.35) / 2, middle.timeOfFlight(), 1e-9);
    assertEquals(1175, TrajectoryCalculator.calculatePassing(turret, pose).wheelRPM());
    assertEquals(
        2775,
        TrajectoryCalculator.calculatePassing(turret.plus(new Translation2d(20, 0)), pose)
            .wheelRPM());
    // Hub lookup retains its existing scale; passing parameters do not modify that table.
    assertEquals(1900 * 0.6, TrajectoryCalculator.calculateRPM(new Translation2d(3, 0), pose));
    assertEquals(-0.5, TrajectoryCalculator.calculateHoodAngle(new Translation2d(3, 0), pose));
  }

  @Test
  void starterTableMatchesDocumentedBallisticAssumptions() {
    // Independent calculation checks the table's provenance, not real-world accuracy.
    double angle = Math.toRadians(55);
    double releaseHeight = 0.40;
    double ballRadius = 5.91 * 0.0254 / 2;
    double hubHeight = FieldConstants.Hub.height + ballRadius;
    double referenceSpeed =
        Math.sqrt(
            9.81
                * 9
                / (2
                    * Math.pow(Math.cos(angle), 2)
                    * (3 * Math.tan(angle) - (hubHeight - releaseHeight))));
    Pose2d pose = Pose2d.kZero;
    Translation2d turret =
        pose.transformBy(GeomUtil.toTransform2d(TurretConstants.kRobotToTurret)).getTranslation();
    for (double distance : new double[] {2, 3, 4, 6, 8, 10}) {
      double speed =
          Math.sqrt(
              9.81
                  * distance
                  * distance
                  / (2
                      * Math.pow(Math.cos(angle), 2)
                      * (distance * Math.tan(angle) - (ballRadius - releaseHeight))));
      var params =
          TrajectoryCalculator.calculatePassing(turret.plus(new Translation2d(distance, 0)), pose);
      assertEquals(1900 * speed / referenceSpeed, params.wheelRPM(), 12.5);
      assertEquals(distance / (speed * Math.cos(angle)), params.timeOfFlight(), 0.005);
      assertEquals(-0.5, params.hoodAngle());
    }
  }
}
