// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wiring;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOAddressable;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOLimelight;
import java.util.function.Supplier;

/** Selects the IO adapters connected to the physical robot. */
public final class RealRobotWiring implements RobotWiring {
  /** Creates the stateless physical-robot wiring selector. */
  public RealRobotWiring() {}

  /** Returns the physical Pigeon 2 gyro adapter. */
  @Override
  public GyroIO createGyro() {
    return new GyroIOPigeon2();
  }

  /** Returns the physical front-left swerve module adapter. */
  @Override
  public ModuleIO createFrontLeftModule() {
    return new ModuleIOTalonFX(TunerConstants.FrontLeft);
  }

  /** Returns the physical front-right swerve module adapter. */
  @Override
  public ModuleIO createFrontRightModule() {
    return new ModuleIOTalonFX(TunerConstants.FrontRight);
  }

  /** Returns the physical back-left swerve module adapter. */
  @Override
  public ModuleIO createBackLeftModule() {
    return new ModuleIOTalonFX(TunerConstants.BackLeft);
  }

  /** Returns the physical back-right swerve module adapter. */
  @Override
  public ModuleIO createBackRightModule() {
    return new ModuleIOTalonFX(TunerConstants.BackRight);
  }

  /** Returns the physical Talon FX indexer adapter. */
  @Override
  public IndexerIO createIndexer() {
    return new IndexerIOTalonFX();
  }

  /** Returns the physical Talon FX intake adapter. */
  @Override
  public IntakeIO createIntake() {
    return new IntakeIOTalonFX();
  }

  /** Returns the physical addressable LED adapter. */
  @Override
  public LedsIO createLeds() {
    return new LedsIOAddressable();
  }

  /** Returns the physical Spark MAX turret adapter. */
  @Override
  public TurretIO createTurret() {
    return new TurretIOSparkMax() {};
  }

  /** Returns the physical Spark MAX hood adapter. */
  @Override
  public HoodIO createHood() {
    return new HoodIOSparkMax();
  }

  /** Returns the physical Talon FX flywheel adapter. */
  @Override
  public FlywheelIO createFlywheel() {
    return new FlywheelIOTalonFX();
  }

  /** Returns the two Limelight adapters in their existing processing order. */
  @Override
  public CameraIO[] createCameras(Supplier<Rotation2d> robotRotationSupplier) {
    return new CameraIO[] {
      new CameraIOLimelight("limelight-front", robotRotationSupplier),
      new CameraIOLimelight("limelight-one", robotRotationSupplier)
    };
  }
}
