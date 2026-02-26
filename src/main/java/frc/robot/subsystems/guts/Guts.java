// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.guts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * This class updates and stores the values of the inputs periodically, and contains commands to run
 * the gut motor forward and backward.
 *
 * @author Ryan Hefferon
 */
public class Guts extends SubsystemBase {

  private final GutSide side;
  private final GutsIO io;
  private GutsIOInputsAutoLogged inputs = new GutsIOInputsAutoLogged();
  private final double speed;

  /** Creates a new Guts. */
  public Guts(GutSide side, GutsIO io) {
    this.io = io;
    this.side = side;
    speed =
        (side == GutSide.LEFT) ? (GutsConstants.kGutMotorSpeed) : -(GutsConstants.kGutMotorSpeed);
  }

  /** Runs the gut motor forward at 0.5 speed, then stops it when finished. */
  public Command runGutForward() {
    return Commands.runEnd(() -> io.setGutMotorSpeed(speed), () -> io.setGutMotorSpeed(0), this);
  }

  /** Runs the gut motor backward at 0.5 speed, then stops it when finished. */
  public Command runGutBackward() {
    return Commands.runEnd(() -> io.setGutMotorSpeed(-speed), () -> io.setGutMotorSpeed(0), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Guts/" + side.getName(), inputs);
    // This method will be called once per scheduler run
  }

  public enum GutSide {
    LEFT("Left"),
    RIGHT("Right");

    private final String name;

    private GutSide(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }
  }
}
