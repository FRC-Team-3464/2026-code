// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final ShooterSide side;

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  /** Creates a new Hood. */
  public Hood(ShooterSide side, HoodIO io) {
    this.side = side;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(("Hood/" + side.getName()), inputs);

    if (side == ShooterSide.LEFT) {
      RobotVisualizer.getInstance().setLeftHoodAngle(inputs.positionRad);
    } else if (side == ShooterSide.RIGHT) {
      RobotVisualizer.getInstance().setRightHoodAngle(inputs.positionRad);
    }
  }

  /**
   * Sets the hood to the target angle.
   *
   * @param angle The target angle (in radians).
   */
  public void setAngle(double angle) {
    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(angle - inputs.positionRad) < HoodConstants.kAngleTolerance);
    io.setAngle(angle);
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
