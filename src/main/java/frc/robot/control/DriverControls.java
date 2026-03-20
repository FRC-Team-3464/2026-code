package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Direction;

public class DriverControls implements Configurable {
  private final DriverController driver;
  private final DriverController operator;
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;

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
    configureSingleController();
  }

  private void configureDriverControls() {
    driver
        .xSquare()
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().resetRotation(Rotation2d.kZero), drive));
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

    operator.rightBumper().whileTrue(shooter.setFlywheelVelocity(8500));

    operator.rightTrigger().whileTrue(indexer.index());

    operator
        .dPadUp()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  shooter.setHoodOpenLoop(0.05);
                },
                () -> {
                  shooter.setHoodOpenLoop(0);
                }));

    operator
        .dPadDown()
        .whileTrue(
            new StartEndCommand(
                () -> {
                  shooter.setHoodOpenLoop(-0.05);
                },
                () -> {
                  shooter.setHoodOpenLoop(0);
                }));

    operator.leftTrigger().whileTrue(intake.outtake());
    operator.aCross().whileTrue(intake.outtake());
    operator.xSquare().whileTrue(intake.deployOpenLoop());
    operator.yTriangle().whileTrue(intake.retractOpenLoop());
    operator.bCircle().whileTrue(shooter.setFlywheelVelocity(2000).alongWith(indexer.index()));

    // operator.aCross().whileTrue(shooter.shootAtTargetNoRotation(() ->
    // RobotState.getInstance().getTurretTarget()));
    // operator.aCross().and(shooter::readyToShoot).whileTrue(indexer.index());
  }

  private void configureSingleController() {

    driver.rightBumper().whileTrue(shooter.setFlywheelVelocity(3000));
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
    
  }
}
