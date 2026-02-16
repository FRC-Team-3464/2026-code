package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class TrajectoryCalculator {
  private static final InterpolatingTreeMap<Double, ShooterParams> shooterTable =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParams::interpolate);

  // TODO update values
  static {
    shooterTable.put(1.5, new ShooterParams(2800.0, 35.0)); // Meters, RPM, Degrees
    shooterTable.put(2.0, new ShooterParams(3100.0, 38.0));
    shooterTable.put(2.5, new ShooterParams(3400.0, 42.0));
    shooterTable.put(3.0, new ShooterParams(3650.0, 46.0));
    shooterTable.put(3.5, new ShooterParams(3900.0, 50.0));
    shooterTable.put(4.0, new ShooterParams(4100.0, 54.0));
    shooterTable.put(4.5, new ShooterParams(4350.0, 58.0));
    shooterTable.put(5.0, new ShooterParams(4550.0, 62.0));
  }

  // public static ShooterParams calculate(Supplier<Pose2d> robotPoseSupplier,
  // Supplier<ChassisSpeeds> robotSpeeds) {

  //   Pose2d currPose = robotPoseSupplier.get();
  //   ChassisSpeeds robotRelativeVel = robotSpeeds.get();

  // }

  public record ShooterParams(double wheelRPM, double hoodAngle)
      implements Interpolatable<ShooterParams> {

    @Override
    public ShooterParams interpolate(ShooterParams other, double t) {
      return new ShooterParams(
          wheelRPM + (other.wheelRPM - wheelRPM) * t,
          hoodAngle + (other.hoodAngle - hoodAngle) * t);
    }
  }
}
