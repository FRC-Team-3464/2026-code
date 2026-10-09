package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.visualization.RobotCadModelGeometry;
import org.littletonrobotics.junction.Logger;

/**
 * The RobotVisualizer class helps record mechanism positions with AdvantageKit. We can attach CAD
 * models of each component to the posted positions to see what our robot actually looked like. <br>
 * None of the methods in this class actually change robot behavior. They just help record it.
 */
public class RobotVisualizer {
  // Makes one single RobotVisualizer object that holds all data (cannot be modified directly, must
  // use helper methods)
  private static RobotVisualizer instance;

  /** */
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
    Pose3d[] components = getComponentPoses();
    Logger.recordOutput(key + "/Components", components);
  }

  /** Returns robot-relative poses in asset order: turret, then hood. */
  public Pose3d[] getComponentPoses() {
    // Tracking commands the negative robot-relative bearing, so display yaw negates feedback.
    // CAD geometry is display-only; it does not replace the aiming calibration.
    Pose3d turretPose =
        new Pose3d(
            RobotCadModelGeometry.TURRET_PIVOT,
            new Rotation3d(0.0, 0.0, -turretAngle.getRadians()));

    // Negate feedback so negative hood readings raise the CAD hood about its front pivot.
    // This display-only sign does not change motor commands. Zero restores the exported CAD pose.
    // TODO: Verify the sign on hardware and calibrate zero and scale from encoder readings.
    Pose3d hoodPose =
        turretPose.transformBy(
            new Transform3d(
                RobotCadModelGeometry.HOOD_OFFSET,
                new Rotation3d(0.0, -hoodAngle, RobotCadModelGeometry.CAD_TURRET_YAW_RAD)));

    return new Pose3d[] {turretPose, hoodPose};
  }

  /**
   * Gets the turret angle.
   *
   * @return A Rotation2d object representing the turret angle.
   */
  public Rotation2d getTurretAzimuthAngle() {
    return turretAngle;
  }

  /**
   * Sets the turret angle.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setTurretAzimuthAngle(Rotation2d angle) {
    turretAngle = angle;
  }

  /**
   * Gets the hood angle.
   *
   * @return A double representing the hood angle in radians.
   */
  public double getTurretHoodAngle() {
    return hoodAngle;
  }

  /**
   * Sets the hood angle in radians.
   *
   * @param angle A Rotation2d object to be inserted in the angles array.
   */
  public void setTurretHoodAngle(double angle) {
    hoodAngle = angle;
  }
}
