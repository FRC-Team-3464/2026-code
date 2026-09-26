package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.Direction;
import java.util.function.Supplier;

public class DriverControls implements Configurable {
  private static final double MANUAL_HOOD_DUTY_CYCLE = 0.05;

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
    driver
        .xSquare()
        .onTrue(
            Commands.runOnce(() -> RobotState.getInstance().resetRotation(Rotation2d.kZero))
                .alongWith(drive.zeroYaw()));
    driver.bCircle().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver.dPadUp().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTH));
    driver.dPadUpLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHWEST));
    driver.dPadUpRight().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHEAST));
    driver.dPadLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.WEST));
    driver.dPadRight().whileTrue(DriveCommands.crabWalk(drive, Direction.EAST));
    driver.dPadDownLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHWEST));
    driver.dPadDownRight().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHEAST));
    driver.dPadDown().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTH));

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
    operator.leftBumper().and(operator.leftTrigger().negate()).whileTrue(intake.intake());

    Supplier<Translation2d> targetSupplier = () -> RobotState.getInstance().getShooterTarget();
    Trigger rightBumper = operator.rightBumper();
    Trigger trackingRequested = rightBumper.and(DriverStation::isTeleopEnabled);
    Trigger dPadUp = operator.dPadUp().and(DriverStation::isTeleopEnabled);
    Trigger dPadDown = operator.dPadDown().and(DriverStation::isTeleopEnabled);
    Trigger manualHoodInputInactive = dPadUp.or(dPadDown).negate();
    Trigger automaticHood =
        trackingRequested.and(manualHoodInputInactive.and(() -> !manualHoodOverrideActive));

    // RB controls the three shooter mechanisms independently. Therefore, a D-pad command can take
    // ownership of the hood without cancelling the turret or flywheel commands. All three triggers
    // include isTeleopEnabled so held controls cannot compete with autonomous commands.
    trackingRequested.whileTrue(shooter.trackTargetTurret(targetSupplier));
    automaticHood.whileTrue(shooter.trackTargetHood(targetSupplier));
    trackingRequested.whileTrue(shooter.trackTargetFlywheel(targetSupplier));

    // Clear the latch when RB is released or teleop ends. This may run while disabled so an old
    // manual angle cannot survive a mode change and suppress automatic aiming later.
    trackingRequested.onFalse(
        Commands.runOnce(() -> manualHoodOverrideActive = false).ignoringDisable(true));

    operator.rightTrigger().whileTrue(indexer.index());

    dPadUp.whileTrue(manualHood(MANUAL_HOOD_DUTY_CYCLE, trackingRequested));

    dPadDown.whileTrue(manualHood(-MANUAL_HOOD_DUTY_CYCLE, trackingRequested));

    operator.aCross().whileTrue(intake.outtake());
    operator.xSquare().whileTrue(intake.deployOpenLoop());
    operator.yTriangle().whileTrue(intake.retractOpenLoop());

    operator
        .dPadLeft()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(-0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));
    operator
        .dPadRight()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));

    operator
        .bCircle()
        .onTrue(Commands.runOnce(() -> shooter.getTurret().zero(), shooter.getTurret()));

    // operator.aCross().whileTrue(shooter.shootAtTargetNoRotation(() ->
    // RobotState.getInstance().getTurretTarget()));
    // operator.aCross().and(shooter::readyToShoot).whileTrue(indexer.index());
  }

  /**
   * Moves the hood manually while a D-pad direction is held.
   *
   * <p>The command requires the hood, so it cleanly interrupts automatic hood tracking instead of
   * allowing two commands to write different hood outputs in the same robot loop. The action runs
   * every scheduler cycle so {@link Hood#setManualOutput(double)} can stop motion when a configured
   * travel limit is reached. When the D-pad is released, the hood changes to closed-loop control at
   * its latest measured position.
   *
   * @param output signed manual motor output
   * @param trackingRequested true only while RB is held during enabled teleop
   */
  private Command manualHood(double output, Trigger trackingRequested) {
    Hood hood = shooter.getHood();
    return Commands.runEnd(
        () -> {
          // If RB is held, preserve the manually selected angle after the D-pad is released.
          manualHoodOverrideActive = trackingRequested.getAsBoolean();
          hood.setManualOutput(output);
        },
        () -> {
          hood.holdCurrentPosition();
          // A disable or autonomous transition makes trackingRequested false, so the latch cannot
          // carry a teleop-only override into another robot mode.
          manualHoodOverrideActive = trackingRequested.getAsBoolean();
        },
        hood);
  }

  private void configureSingleController() {

    driver
        .rightBumper()
        .whileTrue(
            shooter.trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne(
                () -> RobotState.getInstance().getShooterTarget()));
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

    driver.bCircle().whileTrue(indexer.index());
    driver.aCross().whileTrue(indexer.indexReverse());

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

    driver.xSquare().whileTrue(intake.retractOpenLoop());
    driver.yTriangle().whileTrue(intake.deployOpenLoop());

    driver.leftTrigger().whileTrue(intake.outtake());
    driver.rightTrigger().whileTrue(intake.intake());

    driver
        .dPadLeft()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(-0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));
    driver
        .dPadRight()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.getTurret().setOpenLoop(0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));

    driver
        .yTriangle()
        .onTrue(Commands.runOnce(() -> shooter.getTurret().zero(), shooter.getTurret()));
  }
}
