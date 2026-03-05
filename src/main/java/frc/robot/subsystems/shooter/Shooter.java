// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.TrajectoryCalculator.ShooterCommand;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Hub;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private final ShooterSide side;

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  /** Creates a new Shooter. */
  public Shooter(ShooterSide side, TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
    this.side = side;
    this.turret = new Turret(side, turretIO);
    this.hood = new Hood(side, hoodIO);
    this.flywheel = new Flywheel(side, flywheelIO);
  }

  @Override
  public void periodic() {
    turret.periodic();
    hood.periodic();
    flywheel.periodic();
  }

  public static Command shootBothAtHub(Shooter leftShooter, Shooter rightShooter) {
    return shootBothAtTarget(
        leftShooter,
        rightShooter,
        () -> AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
  }

  /**
   * Calculate and apply trajectory parameters for both shooters.
   *
   * @param leftShooter The left shooter subsystem.
   * @param rightShooter The right shooter subsystem.
   * @param targetSupplier A supplier for the target.
   * @return A RunCommand applying trajectory parameters to both shooters.
   */
  public static Command shootBothAtTarget(
      Shooter leftShooter, Shooter rightShooter, Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          var cmds = TrajectoryCalculator.calculateBoth(targetSupplier.get());
          leftShooter.applyCommand(cmds.left());
          rightShooter.applyCommand(cmds.right());
        },
        leftShooter,
        rightShooter);
  }

  public static Command shootBothAtTargetNoTurret(
      Shooter leftShooter, Shooter rightShooter, Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          var cmds = TrajectoryCalculator.calculateBoth(targetSupplier.get());
          leftShooter.applyCommandNoRotation(cmds.left());
          rightShooter.applyCommandNoRotation(cmds.right());
        },
        leftShooter,
        rightShooter);
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
    turret.setPosition(cmd.turretAngle());
  }

  public void applyCommandNoRotation(ShooterCommand cmd) {
    flywheel.setVelocity(cmd.wheelRPM());
    hood.setAngle(cmd.hoodAngle());
  }

  public Command shootAtTarget(Supplier<Translation2d> targetSupplier) {
    return Commands.run(
        () -> {
          ShooterCommand cmd = TrajectoryCalculator.calculate(side, targetSupplier.get());
          flywheel.setVelocity(cmd.wheelRPM());
          hood.setAngle(cmd.hoodAngle());
          turret.setPosition(cmd.turretAngle());
        },
        this,
        turret,
        hood,
        flywheel);
  }

  public Command trackTarget(Supplier<Translation2d> targetSupplier) {
    return turret.trackTarget(targetSupplier);
  }

  public Command zeroTurret() {
    return turret.zero();
  }

  public void setFlywheelVelocity(double velocityRPM) {
    flywheel.setVelocity(velocityRPM);
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

  public ShooterSide getSide() {
    return side;
  }

  public enum ShooterSide {
    LEFT("Left"),
    RIGHT("Right");

    private String name;

    private ShooterSide(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }
  }
}
