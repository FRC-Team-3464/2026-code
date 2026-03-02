package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.guts.Guts;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Direction;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriverControls implements Configurable {
  @AutoLogOutput(key = "Control/DriverControls/mode")
  private DriverMode mode = DriverMode.ONE_DRIVER;

  public enum DriverMode {
    ONE_DRIVER,
    TWO_DRIVERS;
  }

  private final DriverController driver;
  private final Joystick operator;
  private final Drive drive;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Guts leftGuts;
  private final Guts rightGuts;
  private final Intake intake;

  public DriverControls(
      DriverController driver,
      Joystick operator,
      Drive drive,
      Shooter leftShooter,
      Shooter rightShooter,
      Guts leftGuts,
      Guts rightGuts,
      Intake intake) {
    this.driver = driver;
    this.operator = operator;
    this.drive = drive;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
    this.leftGuts = leftGuts;
    this.rightGuts = rightGuts;
    this.intake = intake;
  }

  @Override
  public void configure() {

    // Neutral controls (regardless of whether we are in one or two driver mode)
    driver.xSquare().onTrue(Commands.runOnce(drive::zeroYaw));

    driver
        .bCircle()
        .onTrue(
            Commands.runEnd(
                    () -> driver.rumble(RumbleType.kBothRumble, 1),
                    () -> driver.rumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(0.25));

    driver.dPadUp().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTH));
    driver.dPadUpLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHWEST));
    driver.dPadUpRight().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHEAST));
    driver.dPadLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.WEST));
    driver.dPadRight().whileTrue(DriveCommands.crabWalk(drive, Direction.EAST));
    driver.dPadDownLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHWEST));
    driver.dPadDownRight().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHEAST));
    driver.dPadDown().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTH));

    configureOneDriver();
    configureTwoDrivers();
  }

  /*
   * Driver Bindings:
   *
   * <p>LB: Toggle deploy/retract intake LT: Spin intake RB: Shoot LT + A:
   * backspin intake RT:
   * Climb RT + A: Unclimb X: reset Gyro D-Pad: CrabWalk LB + RB + Y: Aux Handoff
   *
   */

  private void configureOneDriver() {

    // driver
    //     .rightBumper()
    //     .and(this::isOneDriver)
    //     .onTrue(Shooter.shootBothAtHub(leftShooter, rightShooter));

    // driver.leftBumper().and(this::isOneDriver).onTrue(intake.deploy().withTimeout(0.5));

    // driver.leftBumper().and(this::isOneDriver).onTrue(intake.retract().withTimeout(0.5));

    // driver.leftTrigger().and(this::isOneDriver).whileTrue(intake.intake());

    // driver.leftTrigger().and(this::isOneDriver).whileTrue(intake.outtake());
  }

  /*
   * <p>Back up Operator Controls:
   *
   * <p>Pancake up + down: Pitch of turrets Pancake left + right: rotation of
   * turrets trigger
   * button: Fires fuel from turrets
   *
   * <p>button 7: deploy intake button 8: run intake button 9: retract intake
   *
   * <p>button 6: climber up button 4: climber down
   *
   * <p>thumb button: Driver Handoff
   */
  private void configureTwoDrivers() {}

  private boolean isOneDriver() {
    return mode == DriverMode.ONE_DRIVER;
  }

  private boolean isTwoDrivers() {
    return mode == DriverMode.TWO_DRIVERS;
  }

  public void setMode(DriverMode mode) {
    this.mode = mode;
    configure();
  }

  public DriverMode getMode() {
    return mode;
  }
}
