import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import java.util.Locale;

/** Exports the compiled shared geometry for the model builder; does not start the robot. */
class ExportGeometry {
  public static void main(String[] args) {
    var turret = TurretConstants.kRobotToTurret.getTranslation();
    var hood = HoodConstants.kTurretToHood.getTranslation();
    System.out.printf(
        Locale.ROOT,
        "{\"turret\":[%.12f,%.12f,%.12f],\"hoodOffset\":[%.12f,%.12f,%.12f]}%n",
        turret.getX(), turret.getY(), turret.getZ(), hood.getX(), hood.getY(), hood.getZ());
  }
}
