package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class DefaultControls implements Configurable {

  private final DriverController driver;
  private final Joystick operator;
  private final Drive drive;
  private final Shooter leftShooter;
  private final Shooter rightShooter;

  /** Creates a new DefaultControls. */
  public DefaultControls(
      DriverController driver,
      Joystick operator,
      Drive drive,
      Shooter leftShooter,
      Shooter rightShooter) {
    this.driver = driver;
    this.operator = operator;
    this.drive = drive;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
  }

  /** Configure all default commands for the subsystems (e.g. includes joystick driving). */
  @Override
  public void configure() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> -driver.getRightX()));
  }
}
