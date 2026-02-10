// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;

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

  public static final int kDriverControllerPort = 0;
  public static final int kAuxControllerPort = 1;

  public static boolean kDisableHAL = false;

  public static void disableHAL() {
    kDisableHAL = true;
  }

  public static final class DriveConstants {
    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(Drive.getModuleTranslations());

    public static final double kOdometryFrequency =
        ModuleConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double kDriveBaseRadius =
        Math.max(
            Math.max(
                Math.hypot(
                    ModuleConstants.FrontLeft.LocationX, ModuleConstants.FrontLeft.LocationY),
                Math.hypot(
                    ModuleConstants.FrontRight.LocationX, ModuleConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(ModuleConstants.BackLeft.LocationX, ModuleConstants.BackLeft.LocationY),
                Math.hypot(
                    ModuleConstants.BackRight.LocationX, ModuleConstants.BackRight.LocationY)));

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(
              ModuleConstants.FrontLeft.LocationX, ModuleConstants.FrontLeft.LocationY),
          new Translation2d(
              ModuleConstants.FrontRight.LocationX, ModuleConstants.FrontRight.LocationY),
          new Translation2d(ModuleConstants.BackLeft.LocationX, ModuleConstants.BackLeft.LocationY),
          new Translation2d(
              ModuleConstants.BackRight.LocationX, ModuleConstants.BackRight.LocationY)
        };

    // TODO: Update for robot
    // PathPlanner config constants
    public static final double kRobotMassKG = 74.088;
    public static final double kRobotMOI = 6.883;
    /** Coefficient of friction */
    public static final double kWheelCOF = 1.2;

    public static final RobotConfig kPathplannerConfig =
        new RobotConfig(
            kRobotMOI,
            kRobotMOI,
            new ModuleConfig(
                ModuleConstants.FrontLeft.WheelRadius,
                ModuleConstants.kSpeedAt12Volts.in(MetersPerSecond),
                kWheelCOF,
                DCMotor.getKrakenX60(1)
                    .withReduction(ModuleConstants.FrontLeft.DriveMotorGearRatio),
                ModuleConstants.FrontLeft.SlipCurrent,
                1),
            kModuleTranslations);

    public static final class ModuleConstants {
      // Both sets of gains need to be tuned to your individual robot.

      // The steer motor uses any SwerveModule.SteerRequestType control request with
      // the
      // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
      // TODO: Update for robot
      private static final Slot0Configs steerGains =
          new Slot0Configs()
              .withKP(100)
              .withKI(0)
              .withKD(0.5)
              .withKS(0.1)
              .withKV(1.91)
              .withKA(0)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      // When using closed-loop control, the drive motor uses the control
      // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
      // TODO: Update for robot
      private static final Slot0Configs driveGains =
          new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

      // The closed-loop output type to use for the steer motors;
      // This affects the PID/FF gains for the steer motors
      private static final ClosedLoopOutputType kSteerClosedLoopOutput =
          ClosedLoopOutputType.Voltage;
      // The closed-loop output type to use for the drive motors;
      // This affects the PID/FF gains for the drive motors
      private static final ClosedLoopOutputType kDriveClosedLoopOutput =
          ClosedLoopOutputType.Voltage;

      // The type of motor used for the drive motor
      private static final DriveMotorArrangement kDriveMotorType =
          DriveMotorArrangement.TalonFX_Integrated;
      // The type of motor used for the drive motor
      private static final SteerMotorArrangement kSteerMotorType =
          SteerMotorArrangement.TalonFX_Integrated;

      // The remote sensor feedback type to use for the steer motors;
      // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
      // RemoteCANcoder
      private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

      // The stator current at which the wheels start to slip;
      // This needs to be tuned to your individual robot
      // TODO: Update for robot
      private static final Current kSlipCurrent = Amps.of(120.0);

      // Initial configs for the drive and steer motors and the azimuth encoder; these
      // cannot be null.
      // Some configs will be overwritten; check the `with*InitialConfigs()` API
      // documentation.
      private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      private static final TalonFXConfiguration steerInitialConfigs =
          new TalonFXConfiguration()
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      // Swerve azimuth does not require much torque output, so we can set a
                      // relatively
                      // low
                      // stator current limit to help avoid brownouts without impacting performance.
                      .withStatorCurrentLimit(Amps.of(60))
                      .withStatorCurrentLimitEnable(true));
      private static final CANcoderConfiguration encoderInitialConfigs =
          new CANcoderConfiguration();
      // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
      private static final Pigeon2Configuration pigeonConfigs = null;

      // CAN bus that the devices are located on;
      // All swerve devices must share the same CAN bus
      public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

      // Theoretical free speed (m/s) at 12 V applied output;
      // This needs to be tuned to your individual robot
      // TODO: Update for robot
      public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

      // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
      // This may need to be tuned to your individual robot
      // TODO: Update for robot
      private static final double kCoupleRatio = 3.8181818181818183;
      // TODO: Update for robot
      private static final double kDriveGearRatio = 7.363636363636365;
      private static final double kSteerGearRatio = 15.42857142857143;
      private static final Distance kWheelRadius = Inches.of(2.167);
      // TODO: Update for robot
      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = true;
      // TODO: Update for robot
      private static final int kPigeonId = 1;

      // These are only used for simulation
      private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
      private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
      // Simulated voltage necessary to overcome friction
      private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
      private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

      public static final SwerveDrivetrainConstants DrivetrainConstants =
          new SwerveDrivetrainConstants()
              .withCANBusName(kCANBus.getName())
              .withPigeon2Id(kPigeonId)
              .withPigeon2Configs(pigeonConfigs);

      private static final SwerveModuleConstantsFactory<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          ConstantCreator =
              new SwerveModuleConstantsFactory<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                  .withDriveMotorGearRatio(kDriveGearRatio)
                  .withSteerMotorGearRatio(kSteerGearRatio)
                  .withCouplingGearRatio(kCoupleRatio)
                  .withWheelRadius(kWheelRadius)
                  .withSteerMotorGains(steerGains)
                  .withDriveMotorGains(driveGains)
                  .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                  .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                  .withSlipCurrent(kSlipCurrent)
                  .withSpeedAt12Volts(kSpeedAt12Volts)
                  .withDriveMotorType(kDriveMotorType)
                  .withSteerMotorType(kSteerMotorType)
                  .withFeedbackSource(kSteerFeedbackType)
                  .withDriveMotorInitialConfigs(driveInitialConfigs)
                  .withSteerMotorInitialConfigs(steerInitialConfigs)
                  .withEncoderInitialConfigs(encoderInitialConfigs)
                  .withSteerInertia(kSteerInertia)
                  .withDriveInertia(kDriveInertia)
                  .withSteerFrictionVoltage(kSteerFrictionVoltage)
                  .withDriveFrictionVoltage(kDriveFrictionVoltage);

      // TODO: Update for robot
      // Front Left
      private static final int kFrontLeftDriveMotorId = 3;
      private static final int kFrontLeftSteerMotorId = 2;
      private static final int kFrontLeftEncoderId = 1;
      private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
      private static final boolean kFrontLeftSteerMotorInverted = true;
      private static final boolean kFrontLeftEncoderInverted = false;

      private static final Distance kFrontLeftXPos = Inches.of(10);
      private static final Distance kFrontLeftYPos = Inches.of(10);
      // TODO: Update for robot
      // Front Right
      private static final int kFrontRightDriveMotorId = 1;
      private static final int kFrontRightSteerMotorId = 0;
      private static final int kFrontRightEncoderId = 0;
      private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
      private static final boolean kFrontRightSteerMotorInverted = true;
      private static final boolean kFrontRightEncoderInverted = false;

      private static final Distance kFrontRightXPos = Inches.of(10);
      private static final Distance kFrontRightYPos = Inches.of(-10);
      // TODO: Update for robot
      // Back Left
      private static final int kBackLeftDriveMotorId = 7;
      private static final int kBackLeftSteerMotorId = 6;
      private static final int kBackLeftEncoderId = 3;
      private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
      private static final boolean kBackLeftSteerMotorInverted = true;
      private static final boolean kBackLeftEncoderInverted = false;

      private static final Distance kBackLeftXPos = Inches.of(-10);
      private static final Distance kBackLeftYPos = Inches.of(10);
      // TODO: Update for robot
      // Back Right
      private static final int kBackRightDriveMotorId = 5;
      private static final int kBackRightSteerMotorId = 4;
      private static final int kBackRightEncoderId = 2;
      private static final Angle kBackRightEncoderOffset = Rotations.of(0.17236328125);
      private static final boolean kBackRightSteerMotorInverted = true;
      private static final boolean kBackRightEncoderInverted = false;

      private static final Distance kBackRightXPos = Inches.of(-10);
      private static final Distance kBackRightYPos = Inches.of(-10);

      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          FrontLeft =
              ConstantCreator.createModuleConstants(
                  kFrontLeftSteerMotorId,
                  kFrontLeftDriveMotorId,
                  kFrontLeftEncoderId,
                  kFrontLeftEncoderOffset,
                  kFrontLeftXPos,
                  kFrontLeftYPos,
                  kInvertLeftSide,
                  kFrontLeftSteerMotorInverted,
                  kFrontLeftEncoderInverted);
      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          FrontRight =
              ConstantCreator.createModuleConstants(
                  kFrontRightSteerMotorId,
                  kFrontRightDriveMotorId,
                  kFrontRightEncoderId,
                  kFrontRightEncoderOffset,
                  kFrontRightXPos,
                  kFrontRightYPos,
                  kInvertRightSide,
                  kFrontRightSteerMotorInverted,
                  kFrontRightEncoderInverted);
      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          BackLeft =
              ConstantCreator.createModuleConstants(
                  kBackLeftSteerMotorId,
                  kBackLeftDriveMotorId,
                  kBackLeftEncoderId,
                  kBackLeftEncoderOffset,
                  kBackLeftXPos,
                  kBackLeftYPos,
                  kInvertLeftSide,
                  kBackLeftSteerMotorInverted,
                  kBackLeftEncoderInverted);
      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          BackRight =
              ConstantCreator.createModuleConstants(
                  kBackRightSteerMotorId,
                  kBackRightDriveMotorId,
                  kBackRightEncoderId,
                  kBackRightEncoderOffset,
                  kBackRightXPos,
                  kBackRightYPos,
                  kInvertRightSide,
                  kBackRightSteerMotorInverted,
                  kBackRightEncoderInverted);
    }
  }

  public static final class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class ShooterConstants {

    public static final class TurretConstants {
      public static final double kGearRatio = 10 / 1;
      public static final double kMinTurretAngleRad = Units.degreesToRadians(-180);
      public static final double kMaxTurretAngleRad = Units.degreesToRadians(180);

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
      public static final double kMaxAngleRad = Units.degreesToRadians(40);
    }

    public static final class FlywheelConstants {
      public static final double kGearRatio = 300;
      public static final double kSpeedTolerance = 25.0;

      public static final int kLeftFlywheelID = 2;

      public static final Slot0Configs kGains = new Slot0Configs().withKP(1).withKD(0).withKS(0);
      public static final MotorOutputConfigs kOutputConfigs =
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Coast)
              .withInverted(InvertedValue.Clockwise_Positive);
    }
  }
}
