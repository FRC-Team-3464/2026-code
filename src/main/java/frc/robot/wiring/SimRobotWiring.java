// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wiring;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.CameraIO;
import java.util.function.Supplier;

/** Selects the IO adapters used by desktop physics simulation. */
public final class SimRobotWiring implements RobotWiring {
  /** Creates the stateless simulation wiring selector. */
  public SimRobotWiring() {}

  /** Returns an empty gyro adapter because simulated heading is not modeled yet. */
  @Override
  public GyroIO createGyro() {
    return new GyroIO() {};
  }

  /** Returns the simulated front-left swerve module adapter. */
  @Override
  public ModuleIO createFrontLeftModule() {
    return new ModuleIOSim(TunerConstants.FrontLeft);
  }

  /** Returns the simulated front-right swerve module adapter. */
  @Override
  public ModuleIO createFrontRightModule() {
    return new ModuleIOSim(TunerConstants.FrontRight);
  }

  /** Returns the simulated back-left swerve module adapter. */
  @Override
  public ModuleIO createBackLeftModule() {
    return new ModuleIOSim(TunerConstants.BackLeft);
  }

  /** Returns the simulated back-right swerve module adapter. */
  @Override
  public ModuleIO createBackRightModule() {
    return new ModuleIOSim(TunerConstants.BackRight);
  }

  /** Returns the current no-op indexer simulation adapter. */
  @Override
  public IndexerIO createIndexer() {
    return new IndexerIOSim();
  }

  /** Returns the current no-op intake simulation adapter. */
  @Override
  public IntakeIO createIntake() {
    return new IntakeIOSim();
  }

  /** Returns the hardware-free LED simulation adapter. */
  @Override
  public LedsIO createLeds() {
    return new LedsIOSim();
  }

  /** Returns the simulated turret adapter. */
  @Override
  public TurretIO createTurret() {
    return new TurretIOSim();
  }

  /** Returns the simulated hood adapter. */
  @Override
  public HoodIO createHood() {
    return new HoodIOSim();
  }

  /** Returns the simulated flywheel adapter. */
  @Override
  public FlywheelIO createFlywheel() {
    return new FlywheelIOSim();
  }

  /** Returns no cameras because the current SIM mode does not construct vision. */
  @Override
  public CameraIO[] createCameras(Supplier<Rotation2d> robotRotationSupplier) {
    return new CameraIO[0];
  }
}
