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
    public static final double kGearRatio = 10 / 1; // Motor / Turret
    public static final double kMinTurretAngleRad = Units.degreesToRadians(-90);
    public static final double kMaxTurretAngleRad = Units.degreesToRadians(90);
    public static final double kAngleTolerance = Units.degreesToRadians(2);

    public static final double kLeftMotorId = 12;
    public static final double kRightMotorId = 13;

    // +X = Forward, +Y = Left
    public static final Transform3d kRobotToLeftTurret =
        new Transform3d(Inches.of(3.749), Inches.of(8.186), Inches.of(13.401), Rotation3d.kZero);

    public static final Transform3d kRobotToRightTurret =
        new Transform3d(Inches.of(3.749), Inches.of(-8.314), Inches.of(13.401), Rotation3d.kZero);
  }

  public static final class HoodConstants {
    public static final double kTurretToHoodInches = 1.878;
    public static final double kGearRatio = 100 / 1;

    public static final double kLeftHoodID = -1;
    public static final double kRightHoodID = -1;

    public static final double kAngleTolerance = Units.degreesToRadians(5);

    public static final Transform3d kRobotToLeftHood =
        new Transform3d(
            Inches.of(7.268715), Meters.of(0.20792316), Inches.of(16.018516), Rotation3d.kZero);

    public static final Transform3d kRobotToRightHood =
        new Transform3d(
            Inches.of(-7.270121),
            Inches.of(-(12.062888 - (7.5 / 2.0))),
            Inches.of(16.018516),
            Rotation3d.kZero);

    public static final Transform3d kLeftTurretToLeftHood =
        GeomUtil.toPose3d(HoodConstants.kRobotToLeftHood)
            .minus(
                GeomUtil.toPose3d(TurretConstants.kRobotToLeftTurret)
                    .plus(
                        new Transform3d(
                            Inches.of(7.268715), Inches.of(0), Inches.of(0), new Rotation3d())));

    public static final Transform3d kRightTurretToRightHood =
        GeomUtil.toPose3d(HoodConstants.kRobotToRightHood)
            .minus(
                GeomUtil.toPose3d(TurretConstants.kRobotToRightTurret)
                    .plus(
                        new Transform3d(
                            Inches.of(-7.270121), Inches.of(0), Inches.of(0), new Rotation3d())));

    public static final double kMinAngleRad = Units.degreesToRadians(0);
    public static final double kMaxAngleRad = Units.degreesToRadians(30);
  }

  public static final class FlywheelConstants {
    public static final double kGearRatio = 300;
    public static final double kSpeedTolerance = 25.0;

    public static final int kLeftFlywheelID = -1;
    public static final int kRightFlywheelID = -1;

    public static final Slot0Configs kGains =
        new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0.1).withKV(0).withKA(0);
    public static final MotorOutputConfigs kOutputConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
  }
}
