package frc.robot.control;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.TrajectoryCalculator;

public class DefaultControls implements Configurable {

  private final DriverController driver;
  private final DriverController operator;
  private final Drive drive;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;

  /** Creates a new DefaultControls. */
  public DefaultControls(
      DriverController driver,
      DriverController operator,
      Drive drive,
      Indexer indexer,
      Intake intake,
      Shooter shooter) {
    this.driver = driver;
    this.operator = operator;
    this.drive = drive;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
  }

  /**
   * Configure all default commands for the subsystems (e.g. includes joystick
   * driving).
   */
  @Override
  public void configure() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    Supplier<Translation2d> targetPoseSupplier = () -> RobotState.getInstance().getTurretTarget();
    // Avoid the trench
    shooter.setHoodDefaultCommand(shooter.trackTargetHood(targetPoseSupplier));
    shooter.setTurretDefaultCommand(
        shooter.trackTargetTurret(targetPoseSupplier));
  }
}
