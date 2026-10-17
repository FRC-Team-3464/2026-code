// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.RobotState.ShotType;
import frc.robot.subsystems.shooter.TrajectoryCalculator.ShooterCommand;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private Turret turret;
  private Hood hood;
  private Flywheel flywheel;

  /** Creates a new Shooter. */
  public Shooter(TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.turret = new Turret(turretIO);
    this.hood = new Hood(hoodIO);
    this.flywheel = new Flywheel(flywheelIO);
  }

  public boolean readyToShoot() {
    return turret.atGoal() && hood.atGoal() && flywheel.atGoal();
  }

  /**
   * Apply a pre-calculated shooter command to this shooter. This does not require the shooter
   * subsystem - use when combining with other shooters.
   *
   * @param cmd The shot parameters to apply.
   */
  public void applyCommand(ShooterCommand cmd) {
    flywheel.setVelocity(cmd.wheelRPM());
    hood.setAngle(cmd.hoodAngle());
    if (turret != null) {
      turret.setPosition(cmd.turretAngle());
    }
  }

  public void applyCommandNoRotation(ShooterCommand cmd) {
    flywheel.setVelocity(cmd.wheelRPM());
    hood.setAngle(cmd.hoodAngle());
  }

  public Command trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne(
      Supplier<Translation2d> targetSupplier) {
    // e
    return trackTargetTurret(targetSupplier)
        .alongWith(trackTargetHood(targetSupplier), trackTargetFlywheel(targetSupplier));
  }

  public Command shootAtTargetRotation(Supplier<Translation2d> targetSupplier) {
    return Commands.runEnd(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculate(targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
          hood.setAngle(cmd.hoodAngle());
          turret.setPosition(cmd.turretAngle());
        },
        () -> {
          flywheel.setOpenLoop(0);
          hood.setOpenLoop(0);
          turret.setOpenLoop(0);
        },
        this,
        turret,
        hood,
        flywheel);
  }

  public Command shootAtTargetNoRotation(Supplier<Translation2d> targetSupplier) {
    return Commands.runEnd(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculate(targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
          hood.setAngle(cmd.hoodAngle());
        },
        () -> {
          flywheel.setOpenLoop(0);
          hood.setOpenLoop(0);
        },
        this,
        hood,
        flywheel);
  }

  public Command trackTargetFlywheel(Supplier<Translation2d> targetSupplier) {
    return Commands.runEnd(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculateStationary(targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
        },
        () -> {
          flywheel.setOpenLoop(0);
        },
        flywheel);
  }

  // Teleop selects the profile each cycle; autonomous commands above keep the hub table.
  public Command trackTeleopTargetFlywheel(Supplier<ShooterTarget> targetSupplier) {
    return Commands.runEnd(
        () -> {
          ShooterTarget target = targetSupplier.get();
          double rpm =
              target.type() == ShotType.PASS
                  ? TrajectoryCalculator.calculatePassing(
                          target.position(), RobotState.getInstance().getEstimatedPose())
                      .wheelRPM()
                  : TrajectoryCalculator.calculateStationary(target.position()).wheelRPM();
          flywheel.setVelocity(rpm);
        },
        () -> flywheel.setOpenLoop(0),
        flywheel);
  }

  public Command trackTeleopTargetHood(Supplier<ShooterTarget> targetSupplier) {
    return Commands.run(
        () -> {
          ShooterTarget target = targetSupplier.get();
          var pose = RobotState.getInstance().getEstimatedPose();
          double angle =
              target.type() == ShotType.PASS
                  ? TrajectoryCalculator.calculatePassing(target.position(), pose).hoodAngle()
                  : TrajectoryCalculator.calculateHoodAngle(target.position(), pose);
          hood.setAngle(angle);
        },
        hood);
  }

  public Command trackTeleopTargetTurret(Supplier<ShooterTarget> targetSupplier) {
    return trackTargetTurret(() -> targetSupplier.get().position());
  }

  public Command hoodDown() {
    return hood.down();
  }

  public Command trackTargetHood(Supplier<Translation2d> targetSupplier) {
    return hood.trackTarget(targetSupplier);
  }

  public Command trackTargetTurret(Supplier<Translation2d> targetSupplier) {
    return turret.trackTarget(targetSupplier);
  }

  public Command setFlywheelVelocity(double velocityRPM) {
    return flywheel.runVelocity(velocityRPM);
  }

  public void setHoodAngle(double angle) {
    hood.setAngle(angle);
  }

  public void setTurretPosition(Rotation2d position) {
    turret.setPosition(position);
  }

  public void setFlywheelOpenLoop(double output) {
    flywheel.setOpenLoop(output);
  }

  public void setHoodOpenLoop(double output) {
    hood.setOpenLoop(output);
  }

  public void setTurretOpenLoop(double output) {
    turret.setOpenLoop(output);
  }

  public void setTurretDefaultCommand(Command defaultCommand) {
    turret.setDefaultCommand(defaultCommand);
  }

  public void setHoodDefaultCommand(Command defaultCommand) {
    hood.setDefaultCommand(defaultCommand);
  }

  public void setFlywheelDefaultCommand(Command defaultCommand) {
    flywheel.setDefaultCommand(defaultCommand);
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Turret getTurret() {
    return turret;
  }

  public boolean flywheelAtGoal() {
    return flywheel.atGoal();
  }
}
