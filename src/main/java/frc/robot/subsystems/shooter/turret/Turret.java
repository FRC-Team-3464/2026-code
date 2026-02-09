// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.TurretConstants;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final ShooterSide side;

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Rotation2d targetAngle = Rotation2d.kZero;

  /** Creates a new Turret. */
  public Turret(ShooterSide side, TurretIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(("Turret/" + side.getName()), inputs);

    if (side == ShooterSide.LEFT) {
      RobotVisualizer.getInstance().setLeftTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    } else if (side == ShooterSide.RIGHT) {
      RobotVisualizer.getInstance().setRightTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    }

    Logger.recordOutput(("Turret/" + side.getName() + "/TargetAngle"), targetAngle);
  }

  public Command trackTarget(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> targetSupplier) {

    return Commands.run(
        () -> {
          Translation2d target = targetSupplier.get();
          Pose2d robotPose = robotPoseSupplier.get();

          Translation2d turretOffset =
              (this.side == ShooterSide.LEFT
                  ? TurretConstants.kRobotToLeftTurret.getTranslation().toTranslation2d()
                  : TurretConstants.kRobotToRightTurret.getTranslation().toTranslation2d());

          // Turret position in field coordinates
          Translation2d turretFieldPos =
              robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

          // Vector from turret -> target (field frame)
          Translation2d deltaField = target.minus(turretFieldPos);

          // Convert to robot frame
          Translation2d deltaRobot = deltaField.rotateBy(robotPose.getRotation().unaryMinus());

          // Angle turret should point (robot-relative)
          Rotation2d targetAngle = new Rotation2d(Math.atan2(deltaRobot.getY(), deltaRobot.getX()));

          this.targetAngle = targetAngle;

          io.setPosition(targetAngle);
        },
        this);
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public ShooterSide getSide() {
    return this.side;
  }
}
