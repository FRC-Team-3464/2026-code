// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double kLoopPeriodSeconds = 0.02;

  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class DriveConstants {

    public static class ModuleConfigs {

      public static record ModuleConfig(
          int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {}

      /** Module 0 (front left) configs. */
      public static final ModuleConfig FrontLeft =
          new ModuleConfig(1, 2, 19, Rotation2d.fromDegrees(304.36523 - 180));

      /** Module 1 (front right) configs. */
      public static final ModuleConfig FrontRight =
          new ModuleConfig(2, 4, 20, Rotation2d.fromDegrees(206.455));

      /** Module 2 (back left) configs. */
      public static final ModuleConfig BackLeft =
          new ModuleConfig(5, 6, 21, Rotation2d.fromDegrees(35.419922 + 180));

      /** Module 3 (back right) configs. */
      public static final ModuleConfig BackRight =
          new ModuleConfig(7, 8, 22, Rotation2d.fromDegrees(116.89453));
    }

    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kAngleIdleMode = IdleMode.kBrake;
    public static final double kDrivePower = 1;
    public static final double kAnglePower = .9;

    public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

    // drivetrain constants
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelRadius = kWheelDiameter / 2.0;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    // Swerve kinematics, don't change
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // back left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)); // back right

    // gear ratios
    public static final double kDriveGearRatio = (6.12 / 1.0);
    public static final double kAngleGearRatio = ((150.0 / 7.0) / 1.0);

    // encoder stuff
    // meters per rotation
    public static final double kDriveRevToMeters = kWheelCircumference / (kDriveGearRatio);
    public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60;

    /** The number of degrees that a single rotation of the turn motor turns the // wheel. */
    public static final double kDegreesPerTurnRotation = 360 / kAngleGearRatio;

    // motor inverts, check these
    public static final boolean kAngleMotorInvert = true;
    public static final InvertedValue kDriveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    public static final boolean kCanCoderInvert = false;

    /* Swerve Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 20;
    public static final int kAnglePeakCurrentLimit = 40;
    public static final double kAnglePeakCurrentDuration = 0.1;
    public static final boolean kAngleEnableCurrentLimit = true;

    public static final int kDriveSupplyCurrentLimit = 60;
    public static final boolean kDriveSupplyCurrentLimitEnable = true;
    public static final int kDriveSupplyCurrentThreshold = 60;
    public static final double kDriveSupplyTimeThreshold = 0.1;

    public static final boolean kDriveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double kAngleKP = 0.015;
    public static final double kAngleKI = 0;
    public static final double kAngleKD = 0;
    public static final double kAngleKF = 0;

    /* Drive Motor PID Values */

    public static final double kDriveKP = 0.01;
    public static final double kDriveKI = 0.0;
    public static final double kDriveKD = 0.0;

    public static final double kDriveKS = (0.32 / 12);
    public static final double kDriveKV = (1.988 / 12);
    public static final double kDriveKA = (1.0449 / 12);

    /* Swerve Profiling Values */
    /** Meters per second. */
    public static final double kPhysicalMaxSpeed = 5.0;

    public static final double kMaxTeleDriveSpeed = 4.5;
    /** Radians per second. */
    public static final double kPhysicalMaxAngularSpeed = 2 * 2 * Math.PI;
    /** Radians per second. */
    public static final double kMaxTeleAngularSpeed = kPhysicalMaxAngularSpeed / 2;

    public static final double kMaxAngularAccelerationSpeed = 4 / Math.PI;
    /** Radians per second. */
    public static final double kMaxTeleAngularAccelerationSpeed = kMaxAngularAccelerationSpeed / 2;

    public static final double kDeadband = 0.08;

    public static final Map<Integer, Double> kDistances =
        Map.of(
            0, 0.0,
            1, 1.0,
            2, 2.0,
            3, 3.0,
            4, 4.0);
  }
}
