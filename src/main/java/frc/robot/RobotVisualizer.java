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

  private Rotation2d[] turretAngles = {Rotation2d.kZero, Rotation2d.kZero}; // Left, Right
  private double[] hoodAngles = {0.0, 0.0}; // Left, Right

  private RobotVisualizer() {}

  /**
   * Logs the component poses.
   *
   * @param key A String representing the output location.
   */
  public void log(String key) {
    Pose3d leftTurretPose =
        GeomUtil.toPose3d(TurretConstants.kRobotToLeftTurret)
            .transformBy(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, turretAngles[0].plus(Rotation2d.kPi).getRadians())));
    Pose3d rightTurretPose =
        GeomUtil.toPose3d(TurretConstants.kRobotToRightTurret)
            .transformBy(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, turretAngles[1].plus(Rotation2d.kPi).getRadians())));

    Pose3d leftHoodPose =
        leftTurretPose.transformBy(
            new Transform3d(
                HoodConstants.kLeftTurretToLeftHood.getTranslation(),
                new Rotation3d(0.0, hoodAngles[0], 0.0)));

    Pose3d rightHoodPose =
        rightTurretPose.transformBy(
            new Transform3d(
                HoodConstants.kRightTurretToRightHood.getTranslation(),
                new Rotation3d(0.0, hoodAngles[1], 0.0)));

    Logger.recordOutput(
        key + "/Components", leftTurretPose, rightTurretPose, leftHoodPose, rightHoodPose);
  }

  /**
   * Gets the left turret angle.
   *
   * @return A Rotation2d object representing the left turret angle.
   */
  public Rotation2d getLeftTurretAngle() {
    return turretAngles[0];
  }

  /**
   * Gets the right turret angle.
   *
   * @return A Rotation2d object representing the right turret angle.
   */
  public Rotation2d getRightTurretAngle() {
    return turretAngles[1];
  }

  /**
   * Sets the left turret angle.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setLeftTurretAngle(Rotation2d angle) {
    turretAngles[0] = angle;
  }

  /**
   * Sets the right turret angle.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setRightTurretAngle(Rotation2d angle) {
    turretAngles[1] = angle;
  }

  /**
   * Gets the left hood angle.
   *
   * @return A double representing the left hood angle in radians.
   */
  public double getLeftHoodAngle() {
    return hoodAngles[0];
  }

  /**
   * Gets the left hood angle.
   *
   * @return A double representing the right hood angle in radians.
   */
  public double getRightHoodAngle() {
    return hoodAngles[1];
  }

  /**
   * Sets the left hood angle in radians.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setLeftHoodAngle(double angle) {
    hoodAngles[0] = angle;
  }

  /**
   * Sets the right hood angle in radians.
   *
   * @param angle A double to be inserted in the angles array.
   */
  public void setRightHoodAngle(double angle) {
    hoodAngles[1] = angle;
  }
}
