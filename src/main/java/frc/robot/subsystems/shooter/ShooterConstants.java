package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GeomUtil;

public final class ShooterConstants {
  public static final double kLatencySeconds = 0.05;

  public static final class TurretConstants {
    public static final double kGearRatio = 200.0 / 20.0; // Motor / Turret
    public static final double kMinTurretAngleRad = Units.degreesToRadians(-90);
    public static final double kMaxTurretAngleRad = Units.degreesToRadians(210);
    public static final double kAngleTolerance = Units.degreesToRadians(0.5);

    //
    // +X = Forward, +Y = Left
    public static final Transform3d kRobotToTurret =
        new Transform3d(Inches.of(7.5), Inches.of(-4), Inches.of(14.5), Rotation3d.kZero);
  }

  public static final class HoodConstants {
    public static final double kTurretToHoodInches = 1.878;
    public static final double kGearRatio = 16 / 1;

    public static final double kAngleTolerance = Units.degreesToRadians(1);

    public static final Transform3d kRobotToHood =
        new Transform3d(
            Inches.of(7.268715), Meters.of(0.20792316), Inches.of(16.018516), Rotation3d.kZero);

    public static final Transform3d kTurretToHood =
        GeomUtil.toPose3d(HoodConstants.kRobotToHood)
            .minus(
                GeomUtil.toPose3d(TurretConstants.kRobotToTurret)
                    .plus(
                        new Transform3d(
                            Inches.of(7.268715), Inches.of(0), Inches.of(0), new Rotation3d())));

    public static final double kMaxAngleRad = Units.degreesToRadians(0);
    // TODO: Tune
    public static final double kMinAngleRad = -3.9;
  }

  public static final class FlywheelConstants {
    public static final double kGearRatio = 300;
    public static final double kSpeedTolerance = 25.0;

    public static final Slot0Configs kGains =
        new Slot0Configs()
            .withKP(0.1)
            .withKI(0.1)
            .withKD(0.0025)
            .withKS(0.17 * 12)
            .withKV(0.042)
            .withKA(0);
    public static final MotorOutputConfigs kOutputConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
  }
}
