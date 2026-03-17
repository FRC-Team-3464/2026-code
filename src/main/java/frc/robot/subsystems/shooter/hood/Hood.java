// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.TrajectoryCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double targetAngleRad = 0.0;

  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    RobotVisualizer.getInstance().setTurretHoodAngle(inputs.positionRad);

    io.setAngle(targetAngleRad);
  }

  public Command trackTarget(Supplier<Translation2d> targetSupplier) {

    return Commands.run(
        () -> {
          Translation2d target = targetSupplier.get();
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
          setAngle(TrajectoryCalculator.calculateHoodAngle(target, robotPose));
          Logger.recordOutput("Hood target angle", targetAngleRad);
          Logger.recordOutput("Hood target difference", targetAngleRad - inputs.positionRad);
        },
        this);
  }

  public Command down() {
    return Commands.run(() -> setAngle(0), this);
  }

  /**
   * Sets the hood to the target angle.
   *
   * @param angle The target angle (in radians).
   */
  public void setAngle(double angle) {
    atGoal = atGoalDebouncer.calculate(
        Math.abs(angle - inputs.positionRad) < HoodConstants.kAngleTolerance);
    targetAngleRad = angle;
  }

  public void setOpenLoop(double output) {
    io.setOpenLoop(output);
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
}
