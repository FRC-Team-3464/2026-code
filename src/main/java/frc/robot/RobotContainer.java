// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Hub;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);

  private final Drive drive;
  private final Vision vision;

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
        vision = new Vision(null, null);
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(ModuleConstants.FrontLeft),
                new ModuleIOSim(ModuleConstants.FrontRight),
                new ModuleIOSim(ModuleConstants.BackLeft),
                new ModuleIOSim(ModuleConstants.BackRight));
        vision = new Vision(null, null);
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
        vision = new Vision(null, new CameraIO[] {});
        break;
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver
        .rightBumper()
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

                  return new Rotation2d(Math.atan2(delta.getY(), delta.getX()))
                      .plus(Rotation2d.k180deg); // Because KitBot shooter is on the back
                }));

    driver
        .y()
        .onTrue(
            DriveCommands.turnToPoint(
                drive,
                () -> RobotState.getInstance().getEstimatedPose(),
                () -> Hub.innerCenterPoint.toTranslation2d()));
  }

  public void robotPeriodic() {
    OdometryObservation obs =
        new OdometryObservation(
            Timer.getTimestamp(), drive.getModulePositions(), drive.getRawGyroRotation());
    RobotState.getInstance().addOdometryObservation(obs);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
