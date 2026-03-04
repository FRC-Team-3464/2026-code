// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private final ShooterSide side;

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  private Rotation2d targetAngle = Rotation2d.kZero;

  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  private boolean isZeroed = false;

  /** Creates a new Turret. */
  public Turret(ShooterSide side, TurretIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(("Turret/" + side.getName()), inputs);

    if (inputs.limitTriggered) {
      isZeroed = true;
    }

    if (side == ShooterSide.LEFT) {
      RobotVisualizer.getInstance().setLeftTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    } else if (side == ShooterSide.RIGHT) {
      RobotVisualizer.getInstance().setRightTurretAngle(Rotation2d.fromRadians(inputs.positionRad));
    }

    Logger.recordOutput(("Turret/" + side.getName() + "/TargetAngle"), targetAngle);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public Command trackTarget(Supplier<Translation2d> targetSupplier) {

    return Commands.run(
        () -> {
          Translation2d target = targetSupplier.get();
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

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

          setPosition(targetAngle);
        },
        this);
  }

  public Command zero() {
    return Commands.startEnd(() -> setOpenLoop(0.2), () -> stop()).until(this::isZeroed);
  }

  /**
   * Set the target angle for the turret.
   *
   * <p>This will clamp the angle to be within the maximum and minimum rotation of the turret.
   *
   * @param position A {@link Rotation2d} object representing the target position of the turret.
   */
  public void setPosition(Rotation2d position) {
    if (!isZeroed) return; // safety

    targetAngle = position;

    position =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                position.getRadians(),
                TurretConstants.kMinTurretAngleRad,
                TurretConstants.kMaxTurretAngleRad));

    outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    outputs.closedLoopTarget = position;

    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(position.getRadians() - inputs.positionRad) < TurretConstants.kAngleTolerance);
  }

  public void setOpenLoop(double output) {
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    outputs.openLoopOutput = MathUtil.clamp(output, -1.0, 1.0);
  }

  public void stop() {
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    outputs.openLoopOutput = 0.0;
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean isZeroed() {
    return isZeroed;
  }

  public ShooterSide getSide() {
    return this.side;
  }
}
