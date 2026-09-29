// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wiring;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.vision.CameraIO;
import java.util.function.Supplier;

/**
 * Creates the mode-specific IO adapters used by the shared robot subsystems.
 *
 * <p>Each method creates its adapter on demand so {@code RobotContainer} controls initialization
 * order. Implementations must not eagerly construct devices because SIM must never create physical
 * hardware adapters.
 */
public interface RobotWiring {
  /** Returns the gyro adapter for the selected runtime mode. */
  GyroIO createGyro();

  /** Returns the front-left swerve module adapter. */
  ModuleIO createFrontLeftModule();

  /** Returns the front-right swerve module adapter. */
  ModuleIO createFrontRightModule();

  /** Returns the back-left swerve module adapter. */
  ModuleIO createBackLeftModule();

  /** Returns the back-right swerve module adapter. */
  ModuleIO createBackRightModule();

  /** Returns the indexer adapter for the selected runtime mode. */
  IndexerIO createIndexer();

  /** Returns the intake adapter for the selected runtime mode. */
  IntakeIO createIntake();

  /** Returns the LED adapter for the selected runtime mode. */
  LedsIO createLeds();

  /** Returns the turret adapter for the selected runtime mode. */
  TurretIO createTurret();

  /** Returns the hood adapter for the selected runtime mode. */
  HoodIO createHood();

  /** Returns the flywheel adapter for the selected runtime mode. */
  FlywheelIO createFlywheel();

  /**
   * Returns the camera adapters in their intended processing order.
   *
   * @param robotRotationSupplier supplies the robot heading required by supported camera adapters
   */
  CameraIO[] createCameras(Supplier<Rotation2d> robotRotationSupplier);
}
