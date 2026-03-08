package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.guts.Guts;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Direction;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class DriverControls implements Configurable {
  @AutoLogOutput(key = "Control/DriverControls/mode")
  private DriverMode mode = DriverMode.TWO_DRIVERS;

  public enum DriverMode {
    ONE_DRIVER,
    TWO_DRIVERS;
  }

  private final DriverController driver;
  private final DriverController operator;
  private final Drive drive;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Guts leftGuts;
  private final Guts rightGuts;
  private final Intake intake;

  public DriverControls(
      DriverController driver,
      DriverController operator,
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
    driver.xSquare().onTrue(Commands.runOnce(drive::zeroYaw, drive));
    driver.bCircle().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver.dPadUp().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTH));
    driver.dPadUpLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHWEST));
    driver.dPadUpRight().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHEAST));
    driver.dPadLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.WEST));
    driver.dPadRight().whileTrue(DriveCommands.crabWalk(drive, Direction.EAST));
    driver.dPadDownLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHWEST));
    driver.dPadDownRight().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHEAST));
    driver.dPadDown().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTH));

    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(), // xSupplier
                () -> -driver.getLeftX(), // ySupplier
                () -> {
                  Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
                  Translation2d target =
                      AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

                  Translation2d delta = target.minus(robotPose.getTranslation());

                  return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
                }));

    operator.leftBumper().and(operator.leftTrigger().negate()).whileTrue(intake.intake());

    operator
        .rightBumper()
        .whileTrue(
            rightShooter
                .setFlywheelVelocity(8500)
                .alongWith(leftShooter.setFlywheelVelocity(-8500)));

    operator
        .rightTrigger()
        .whileTrue(leftGuts.runGutForward().alongWith(rightGuts.runGutForward()));

    operator
        .dPadUp()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  leftShooter.setHoodOpenLoop(0.05);
                  rightShooter.setHoodOpenLoop(0.05);
                },
                () -> {
                  leftShooter.setHoodOpenLoop(0);
                  rightShooter.setHoodOpenLoop(0);
                }));

    operator
        .dPadDown()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  leftShooter.setHoodOpenLoop(-0.05);
                  rightShooter.setHoodOpenLoop(-0.05);
                },
                () -> {
                  leftShooter.setHoodOpenLoop(0);
                  rightShooter.setHoodOpenLoop(0);
                }));

    operator.leftTrigger().whileTrue(intake.outtake());
    operator.aCross().whileTrue(intake.outtake());
    operator.xSquare().whileTrue(intake.deploy());
    operator.yTriangle().whileTrue(intake.retract());
    operator
        .bCircle()
        .whileTrue(
            rightShooter
                .setFlywheelVelocity(2000)
                .alongWith(
                    leftShooter.setFlywheelVelocity(-2000),
                    rightGuts.runGutForward(),
                    leftGuts.runGutForward()));
  }

  /*
   * Driver Bindings:
   *
   * LB: Toggle deploy/retract intake
   * LT: Spin intake
   * RB: Shoot
   * LT + A: backspin intake
   * RT: Climb RT + A: Unclimb
   * X: reset Gyro
   * D-Pad: CrabWalk
   * LB + RB + Y: Aux Handoff
   *
   */
  private void configureOneDriver() {}

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
