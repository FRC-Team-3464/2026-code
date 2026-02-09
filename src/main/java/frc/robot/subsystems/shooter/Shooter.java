// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
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

  public Command trackTarget(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {
    return Commands.idle(this).alongWith(turret.trackTarget(robotPoseSupplier, targetSupplier));
  }

  public void setFlywheelVelocity(double velocityRPM) {
    flywheel.setVelocity(velocityRPM);
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
