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
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
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

  public static boolean kDisableHAL = false;

  public static void disableHAL() {
    kDisableHAL = true;
  }

  public class DriveConstants {

    // TunerConstants doesn't include these constants
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
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(ModuleConstants.FrontLeft.DriveMotorGearRatio),
                ModuleConstants.FrontLeft.SlipCurrent,
                1),
            kModuleTranslations);

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

  public class ModuleConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
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
    private static final Slot0Configs driveGains =
        new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

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
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.8181818181818183;

    private static final double kDriveGearRatio = 7.363636363636365;
    private static final double kSteerGearRatio = 15.42857142857143;
    private static final Distance kWheelRadius = Inches.of(2.167);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

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

    // Front Left
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 1;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    // Front Right
    private static final int kFrontRightDriveMotorId = 1;
    private static final int kFrontRightSteerMotorId = 0;
    private static final int kFrontRightEncoderId = 0;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    // Back Left
    private static final int kBackLeftDriveMotorId = 7;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 3;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10);
    private static final Distance kBackLeftYPos = Inches.of(10);

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

    /**
     * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
     * program,.
     */
    // public static CommandSwerveDrivetrain createDrivetrain() {
    // return new CommandSwerveDrivetrain(
    // DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    // }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
      }

      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
       *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          double odometryUpdateFrequency,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            odometryUpdateFrequency,
            modules);
      }

      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
       *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
       * @param odometryStandardDeviation The standard deviation for odometry calculation in the
       *     form [x, y, theta]ᵀ, with units in meters and radians
       * @param visionStandardDeviation The standard deviation for vision calculation in the form
       *     [x, y, theta]ᵀ, with units in meters and radians
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          double odometryUpdateFrequency,
          Matrix<N3, N1> odometryStandardDeviation,
          Matrix<N3, N1> visionStandardDeviation,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            odometryUpdateFrequency,
            odometryStandardDeviation,
            visionStandardDeviation,
            modules);
      }
    }
  }
}
