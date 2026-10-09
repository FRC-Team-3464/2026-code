package frc.robot.control;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultControls implements Configurable {
  private final DriverController driver;
  private final Drive drive;
  private final Shooter shooter;
  private final DoubleSupplier rotationInput;
  private final BooleanSupplier holdHood;

  public DefaultControls(
      DriverController driver,
      Drive drive,
      Shooter shooter,
      DoubleSupplier rotationInput,
      BooleanSupplier holdHood) {
    this.driver = driver;
    this.drive = drive;
    this.shooter = shooter;
    this.rotationInput = rotationInput;
    this.holdHood = holdHood;
  }

  @Override
  public void configure() {
    // Default commands may run outside teleop when no autonomous command owns the subsystem.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> DriverStation.isTeleopEnabled() ? -driver.getLeftY() : 0,
            () -> DriverStation.isTeleopEnabled() ? -driver.getLeftX() : 0,
            () -> DriverStation.isTeleopEnabled() ? rotationInput.getAsDouble() : 0));
    shooter
        .getTurret()
        .setDefaultCommand(
            new RunCommand(() -> shooter.getTurret().setOpenLoop(0), shooter.getTurret()));
    shooter
        .getHood()
        .setDefaultCommand(
            new RunCommand(
                () -> {
                  // Manual movement holds its last angle on release. Preserve it while
                  // aiming/manual control
                  // is held in teleop; otherwise use the existing starting-angle behavior.
                  if (!(DriverStation.isTeleopEnabled() && holdHood.getAsBoolean())) {
                    shooter.getHood().setAngle(0);
                  }
                },
                shooter.getHood()));
  }
}
