package frc.robot.control;

import static frc.robot.subsystems.shooter.ShooterConstants.HoodConstants.kManualDutyCycle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Direction;
import java.util.function.Supplier;

public class DriverControls implements Configurable {
  private final DriverController driver;
  private final DriverController operator;
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  // This latch remembers that the operator manually positioned the hood while holding RB. It keeps
  // automatic aiming from immediately replacing that position when the D-pad is released. Releasing
  // RB clears the latch so the next RB press can start automatic hood aiming again.
  private boolean manualHoodOverrideActive;

  public DriverControls(
      DriverController driver,
      DriverController operator,
      Drive drive,
      Shooter shooter,
      Intake intake,
      Indexer indexer) {
    this.driver = driver;
    this.operator = operator;
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;
  }

  @Override
  public void configure() {
    configureDriverControls();
    configureOperatorControls();
  }

  private void configureDriverControls() {
    // Bindings are polled in autonomous and test too. Gate manual requests at the trigger so they
    // cannot take ownership from autonomous commands. While-held controls activate on teleop entry.
    // Reset references only on a fresh physical press in teleop. Gating the trigger itself would
    // also create a rising edge when teleop starts with the button already held.
    teleopPress(driver.xSquare())
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().resetRotation(Rotation2d.kZero))
                .alongWith(drive.zeroYaw()));
    driver
        .bCircle()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver
        .dPadUp()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.NORTH));
    driver
        .dPadUpLeft()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHWEST));
    driver
        .dPadUpRight()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHEAST));
    driver
        .dPadLeft()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.WEST));
    driver
        .dPadRight()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.EAST));
    driver
        .dPadDownLeft()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHWEST));
    driver
        .dPadDownRight()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHEAST));
    driver
        .dPadDown()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTH));

    // driver
    // .leftBumper()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -driver.getLeftY(), // xSupplier
    // () -> -driver.getLeftX(), // ySupplier
    // () -> {
    // Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    // Translation2d target =
    // AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

    // Translation2d delta = target.minus(robotPose.getTranslation());

    // return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    // }));
  }

  private void configureOperatorControls() {
    // Guard operator bindings here so autonomous can still use the same subsystem commands.
    // Leaving enabled teleop cancels while-held requests through their existing end actions.
    operator
        .leftBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(operator.leftTrigger().negate())
        .whileTrue(intake.intake());

    Supplier<Translation2d> targetSupplier = () -> RobotState.getInstance().getShooterTarget();

    // RB controls the three shooter mechanisms independently. Therefore, a D-pad command can take
    // ownership of the hood without cancelling the turret or flywheel commands. Controller
    // triggers are still polled during autonomous, so these teleop guards prevent held controls
    // from competing with autonomous shooter commands.
    operator
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.trackTargetTurret(targetSupplier));
    operator
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(operator.dPadUp().or(operator.dPadDown()).negate())
        .and(() -> !manualHoodOverrideActive)
        .whileTrue(shooter.trackTargetHood(targetSupplier));
    operator
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.trackTargetFlywheel(targetSupplier));

    // Remember that the operator chose a manual hood angle while tracking. Keeping this controller
    // state here prevents Hood from depending on RB or D-pad inputs.
    operator
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(operator.dPadUp().or(operator.dPadDown()))
        .onTrue(Commands.runOnce(() -> manualHoodOverrideActive = true));

    // Clear the latch when RB is released or teleop ends. This may run while disabled so an old
    // manual angle cannot survive a mode change and suppress automatic aiming later.
    operator
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .onFalse(Commands.runOnce(() -> manualHoodOverrideActive = false).ignoringDisable(true));

    operator.rightTrigger().and(DriverStation::isTeleopEnabled).whileTrue(indexer.index());

    operator
        .dPadUp()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.getHood().manualControl(kManualDutyCycle));
    operator
        .dPadDown()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.getHood().manualControl(-kManualDutyCycle));

    operator.aCross().and(DriverStation::isTeleopEnabled).whileTrue(intake.outtake());
    operator.xSquare().and(DriverStation::isTeleopEnabled).whileTrue(intake.deployOpenLoop());
    operator.yTriangle().and(DriverStation::isTeleopEnabled).whileTrue(intake.retractOpenLoop());

    operator
        .dPadLeft()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(-0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));
    operator
        .dPadRight()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));

    // A held button must not redefine the turret's encoder zero on teleop entry or re-enable.
    teleopPress(operator.bCircle())
        .onTrue(Commands.runOnce(() -> shooter.getTurret().zero(), shooter.getTurret()));

    // operator.aCross().whileTrue(shooter.shootAtTargetNoRotation(() ->
    // RobotState.getInstance().getTurretTarget()));
    // operator.aCross().and(shooter::readyToShoot).whileTrue(indexer.index());
  }

  /**
   * Allows a reset only on a new physical button press during enabled teleop.
   *
   * <p>Capture the button edge before applying the mode condition: enabling teleop while a button
   * is held must not count as a new press. Filtering before scheduling also prevents a blocked
   * reset from claiming its subsystem and interrupting an autonomous command.
   */
  private Trigger teleopPress(Trigger button) {
    return new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), button)
        .rising()
        .castTo(Trigger::new)
        .and(DriverStation::isTeleopEnabled);
  }

  private void configureSingleController() {
    // Preserve teleop-only manual control if this alternative layout is enabled in configure().

    driver
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.trackTarget(() -> RobotState.getInstance().getShooterTarget()));
    // // RB -> Shoot
    // driver
    // .rightBumper()
    // .whileTrue(
    // Commands.runEnd(
    // () -> shooter.setFlywheelOpenLoop(.0175),
    // () -> shooter.setFlywheelOpenLoop(0),
    // shooter));
    // driver
    // .leftBumper()
    // .whileTrue(
    // Commands.runEnd(
    // () -> shooter.setFlywheelOpenLoop(.0185),
    // () -> shooter.setFlywheelOpenLoop(0),
    // shooter));

    driver.bCircle().and(DriverStation::isTeleopEnabled).whileTrue(indexer.index());
    driver.aCross().and(DriverStation::isTeleopEnabled).whileTrue(indexer.indexReverse());

    // driver
    // .aCross()
    // .whileTrue(
    // Commands.runEnd(
    // () -> indexer.setThroatOpenLoop(0.5), () -> indexer.setThroatOpenLoop(0),
    // indexer));
    // // driver
    // // .bCircle()
    // // .whileTrue(
    // // Commands.runEnd(
    // // () -> indexer.setThroatOpenLoop(-0.5),
    // // () -> indexer.setThroatOpenLoop(0),
    // // indexer));

    driver.xSquare().and(DriverStation::isTeleopEnabled).whileTrue(intake.retractOpenLoop());
    driver.yTriangle().and(DriverStation::isTeleopEnabled).whileTrue(intake.deployOpenLoop());

    driver.leftTrigger().and(DriverStation::isTeleopEnabled).whileTrue(intake.outtake());
    driver.rightTrigger().and(DriverStation::isTeleopEnabled).whileTrue(intake.intake());

    driver
        .dPadLeft()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(-0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));
    driver
        .dPadRight()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));

    // A held button must not redefine the turret's encoder zero on teleop entry or re-enable.
    teleopPress(driver.yTriangle())
        .onTrue(Commands.runOnce(() -> shooter.getTurret().zero(), shooter.getTurret()));
  }
}
