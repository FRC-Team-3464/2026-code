package frc.robot.control;

import static frc.robot.subsystems.shooter.ShooterConstants.HoodConstants.kManualDutyCycle;

import edu.wpi.first.math.MathUtil;
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

/** Driving and mechanism bindings for one controller. */
public class SingleDriverControls implements Configurable {
  private final DriverController driver;
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private boolean manualStickMode;
  private boolean rightStickReady;
  // Manual aiming stays selected until the automatic-aim button is released.
  private boolean manualAimOverrideActive;
  private Trigger hoodHold = new Trigger(() -> false);

  public SingleDriverControls(
      DriverController driver, Drive drive, Shooter shooter, Intake intake, Indexer indexer) {
    this.driver = driver;
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;
  }

  @Override
  public void configure() {
    configureMechanismControls();
    configureDriverControls();
  }

  private void configureMechanismControls() {
    hoodHold = driver.rightBumper().or(driver.leftTrigger());
    // Poll before bindings and defaults so both stick roles use the same decision this cycle.
    CommandScheduler.getInstance()
        .getDefaultButtonLoop()
        .bind(
            () -> {
              boolean requestedManual = driver.leftTrigger().getAsBoolean();
              if (!DriverStation.isTeleopEnabled() || requestedManual != manualStickMode) {
                rightStickReady = false;
              }
              manualStickMode = requestedManual;
              if (DriverStation.isTeleopEnabled()
                  && Math.abs(driver.getRightX()) <= 0.1
                  && Math.abs(driver.getRightY()) <= 0.1) {
                rightStickReady = true;
              }
            });

    // Pivot takes precedence over rollers, outtake over collection. Mutually exclusive triggers
    // also let a held LB resume after a pivot/outtake request without relying on binding order.
    driver
        .leftBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.yTriangle().or(driver.startMenu()).negate())
        .and(driver.aCross().negate())
        .whileTrue(intake.intake());
    driver
        .aCross()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.yTriangle().or(driver.startMenu()).negate())
        .whileTrue(intake.outtake());
    driver
        .yTriangle()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.startMenu().negate())
        .whileTrue(intake.deployOpenLoop());
    driver
        .startMenu()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.yTriangle().negate())
        .whileTrue(intake.retractOpenLoop());
    driver.rightTrigger().and(DriverStation::isTeleopEnabled).whileTrue(indexer.index());

    Supplier<Translation2d> targetSupplier = () -> RobotState.getInstance().getShooterTarget();
    // Remember a manual override until the aiming button is released, as in operator controls.
    driver
        .rightBumper()
        .and(driver.leftTrigger())
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> manualAimOverrideActive = true));
    driver
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .onFalse(Commands.runOnce(() -> manualAimOverrideActive = false).ignoringDisable(true));
    driver
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.leftTrigger().negate())
        .and(() -> !manualAimOverrideActive)
        .whileTrue(shooter.trackTargetTurret(targetSupplier));
    driver
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .and(driver.leftTrigger().negate())
        .and(() -> !manualAimOverrideActive)
        .whileTrue(
            shooter
                .trackTargetHood(targetSupplier)
                .finallyDo(
                    () -> {
                      if (DriverStation.isTeleopEnabled()) {
                        shooter.getHood().holdCurrentPosition();
                      } else {
                        shooter.getHood().setOpenLoop(0);
                      }
                    }));
    // The flywheel remains independent of manual turret/hood adjustment, as in dual mode.
    driver
        .rightBumper()
        .and(DriverStation::isTeleopEnabled)
        .whileTrue(shooter.trackTargetFlywheel(targetSupplier));

    driver
        .leftTrigger()
        .and(DriverStation::isTeleopEnabled)
        .and(() -> rightStickReady)
        .and(() -> Math.abs(driver.getRightX()) > 0.1)
        .whileTrue(
            Commands.runEnd(
                () ->
                    shooter
                        .getTurret()
                        .setOpenLoop(MathUtil.applyDeadband(driver.getRightX(), 0.1) * 0.05),
                () -> shooter.getTurret().setOpenLoop(0),
                shooter.getTurret()));
    // Stick up requests the same signed hood output as operator D-pad up.
    driver
        .leftTrigger()
        .and(DriverStation::isTeleopEnabled)
        .and(() -> rightStickReady)
        .and(() -> Math.abs(driver.getRightY()) > 0.1)
        .whileTrue(
            Commands.runEnd(
                () ->
                    shooter
                        .getHood()
                        .setManualOutput(
                            MathUtil.applyDeadband(-driver.getRightY(), 0.1) * kManualDutyCycle),
                shooter.getHood()::holdCurrentPosition,
                shooter.getHood()));
  }

  /** Registers heading reset, X-lock and directional driving buttons. */
  private void configureDriverControls() {
    // Gate manual commands to teleop; resets additionally require a fresh physical button press.
    teleopPress(driver.xSquare()).onTrue(drive.resetHeading());
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
  }

  /**
   * Allows a reset only on a new physical button press during enabled teleop.
   *
   * <p>Capture the button edge before applying the mode condition: enabling teleop while a button
   * is held must not count as a new press. Filtering before scheduling also prevents a blocked
   * reset from claiming its subsystem and interrupting an autonomous command.
   */
  private static Trigger teleopPress(Trigger button) {
    return new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), button)
        .rising()
        .castTo(Trigger::new)
        .and(DriverStation::isTeleopEnabled);
  }

  /** Supplies chassis rotation after the shared stick has returned to neutral. */
  public double rotationInput() {
    return manualStickMode || !rightStickReady ? 0 : -driver.getRightX();
  }

  /** Retains the manual hood angle while aiming or manual adjustment remains selected. */
  public boolean holdHood() {
    return hoodHold.getAsBoolean();
  }
}
