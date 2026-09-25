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
  // IO representation + inputs for the hood
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double targetAngleRad = 0.0;
  private boolean closedLoop = false;

  // Boolean representing if the hood is at its target RPM
  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kRising);

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Typical IO input cycle
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (!closedLoop) {
      // Uses the debouncer to determine if the hood has been at the goal angle for enough time
      atGoal =
          atGoalDebouncer.calculate(
              Math.abs(targetAngleRad - inputs.positionRad) < HoodConstants.kAngleTolerance);
    }
    if (closedLoop) {
      io.setAngle(targetAngleRad);
    }

    // Log the exact mechanism position for visualization in AdvantageScope
    RobotVisualizer.getInstance().setTurretHoodAngle(inputs.positionRad);
  }

  public Command trackTarget(Supplier<Translation2d> targetSupplier) {
    // Run command: Continuously repeats until command ends
    return Commands.run(
        () -> {
          // Get the current target (typically the hub)
          Translation2d target = targetSupplier.get();
          // Get the current robot pose
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
          // Use the lookup table to get the goal angle based on distance to the target
          setAngle(TrajectoryCalculator.calculateHoodAngle(target, robotPose));
          // Record the target angle and difference between positions
          Logger.recordOutput("Hood target angle", targetAngleRad);
          Logger.recordOutput("Hood target difference", targetAngleRad - inputs.positionRad);
        },
        this); // Reference to current subsystem
  }

  /** Move the hood all the way down. */
  public Command down() {
    return Commands.run(() -> setAngle(0), this);
  }

  /**
   * Sets the hood to the target angle.
   *
   * @param angle The target angle (in radians).
   */
  public void setAngle(double angle) {
    closedLoop = true;
    targetAngleRad = angle;
  }

  /** Run the hood motor at the specified open loop value. */
  public void setOpenLoop(double output) {
    closedLoop = false;
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
