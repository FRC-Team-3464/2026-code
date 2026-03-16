package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private static RobotVisualizer instance;

  public static RobotVisualizer getInstance() {
    if (instance == null) {
      instance = new RobotVisualizer();
    }
    return instance;
  }

  private Rotation2d turretAngle = Rotation2d.kZero;
  private double hoodAngle = 0.0;

  private RobotVisualizer() {}

  /**
   * Logs the component poses.
   *
   * @param key A String representing the output location.
   */
  public void log(String key) {
    Pose3d turretPose =
        GeomUtil.toPose3d(TurretConstants.kRobotToTurret)
            .transformBy(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, turretAngle.plus(Rotation2d.kPi).getRadians())));

    Pose3d hoodPose =
        turretPose.transformBy(
            new Transform3d(
                HoodConstants.kTurretToHood.getTranslation(),
                new Rotation3d(0.0, hoodAngle, 0.0)));

    Logger.recordOutput(
        key + "/Components", turretPose, hoodPose);
  }

  /**
   * Gets the left turret angle.
   *
   * @return A Rotation2d object representing the left turret angle.
   */
  public Rotation2d getTurretAzimuthAngle() {
    return turretAngle;
  }

  /**
   * Sets the left turret angle.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setTurretAzimuthAngle(Rotation2d angle) {
    turretAngle = angle;
  }

  /**
   * Gets the left hood angle.
   *
   * @return A double representing the left hood angle in radians.
   */
  public double getTurretHoodAngle() {
    return hoodAngle;
  }

  /**
   * Sets the left hood angle in radians.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setTurretHoodAngle(double angle) {
    hoodAngle = angle;
  }
}
