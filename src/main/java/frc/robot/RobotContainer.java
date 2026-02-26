// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Direction;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Hub;

public class RobotContainer {
  private final CommandXboxController driver =
      new CommandXboxController(Constants.kDriverControllerPort);

  private Drive drive;
  private Shooter leftShooter;
  private Shooter rightShooter;
  private Intake intake;
  // private Vision vision;

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(ModuleConstants.FrontLeft),
                new ModuleIOTalonFX(ModuleConstants.FrontRight),
                new ModuleIOTalonFX(ModuleConstants.BackLeft),
                new ModuleIOTalonFX(ModuleConstants.BackRight));
        // vision = new Vision(null, null);
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(ModuleConstants.FrontLeft),
                new ModuleIOSim(ModuleConstants.FrontRight),
                new ModuleIOSim(ModuleConstants.BackLeft),
                new ModuleIOSim(ModuleConstants.BackRight));
        leftShooter =
            new Shooter(ShooterSide.LEFT, new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
        rightShooter =
            new Shooter(ShooterSide.RIGHT, new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
        // vision = new Vision(null, null);
        break;
      case REPLAY:
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // vision = new Vision(null, new CameraIO[] {});
        break;
    }

    // if (Constants.kCurrentMode == Constants.Mode.REAL) {
    //   try {
    //     Constants.kRobotConfig = RobotConfig.fromGUISettings();
    //   } catch (Exception e) {
    //     // Handle exception as needed
    //     e.printStackTrace();
    //   }
    // }

    // configurePathPlanner();
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    leftShooter.setDefaultCommand(
        leftShooter.trackTarget(
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d())));
    rightShooter.setDefaultCommand(
        rightShooter.trackTarget(
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d())));

    driver.povUp().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTH));
    driver.povUpRight().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHEAST));
    driver.povRight().whileTrue(DriveCommands.crabWalk(drive, Direction.EAST));
    driver.povDownRight().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHEAST));
    driver.povDown().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTH));
    driver.povDownLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.SOUTHWEST));
    driver.povLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.WEST));
    driver.povUpLeft().whileTrue(DriveCommands.crabWalk(drive, Direction.NORTHWEST));

    driver.rightBumper().whileTrue(Shooter.shootBothAtHub(leftShooter, rightShooter));

    driver
        .a()
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

    driver
        .y()
        .onTrue(
            DriveCommands.turnToPoint(
                drive,
                () -> RobotState.getInstance().getEstimatedPose(),
                () -> Hub.innerCenterPoint.toTranslation2d()));
  }

  public void robotContainerPeriodic() {
    OdometryObservation obs =
        new OdometryObservation(
            Timer.getTimestamp(), drive.getModulePositions(), drive.getRawGyroRotation());
    RobotState.getInstance().addOdometryObservation(obs);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void configurePathPlanner() {
    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().setPose(pose),
        () -> RobotState.getInstance().getRobotVelocity(),
        (speeds, feedforwards) -> drive.runVelocity(speeds),
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(0, 0, 0)),
        Constants.kRobotConfig,
        AllianceFlipUtil::shouldFlip,
        drive);

    /** PATHPLANNER COMMANDS */
    NamedCommands.registerCommand(
        "resetGyro",
        new InstantCommand(() -> RobotState.getInstance().resetRotation(Rotation2d.kZero)));

    NamedCommands.registerCommand(
        "scoreBothShooters",
        Shooter.shootBothAtTarget(
            leftShooter,
            rightShooter,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d())));
  }
}
