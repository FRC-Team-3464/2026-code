// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.control.DriverController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class RobotContainer {
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final DriverController operator = new DriverController.XboxDriverController(1);

  private Turret turret;
  private Vision vision;
  private Drive drive;

  private Field2d field2d = new Field2d();

  public RobotContainer() {
    turret = new Turret(ShooterSide.LEFT, new TurretIOSparkMax(ShooterSide.LEFT));
    Supplier<Rotation2d> robotRotationSupplier = () -> RobotState.getInstance().getRotation();
    // vision =
    //     new Vision(
    //         new VisionConsumer() {
    //           public void accept(
    //               Pose2d visionRobotPoseMeters,
    //               double timestampSeconds,
    //               edu.wpi.first.math.Matrix<N3, N1> visionMeasurementStdDevs) {

    //             RobotState.getInstance()
    //                 .addVisionMeasurement(
    //                     new VisionMeasurement(
    //                         timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
    //           }
    //           ;
    //         },
    //         new CameraIOLimelight("limelight-front", robotRotationSupplier),
    //         new CameraIOLimelight("limelight", robotRotationSupplier));
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});

    turret.setDefaultCommand(
        new RunCommand(() -> turret.setPosition(RobotState.getInstance().getRotation()), turret));

    driver
        .rightBumper()
        .whileTrue(
            Commands.runEnd(() -> turret.setOpenLoop(0.1), () -> turret.setOpenLoop(0), turret));

    driver
        .leftBumper()
        .whileTrue(
            Commands.runEnd(() -> turret.setOpenLoop(-0.1), () -> turret.setOpenLoop(0), turret));
  }

  public void robotPeriodic() {
    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                Timer.getTimestamp(),
                new SwerveModulePosition[] {
                  new SwerveModulePosition(
                      Math.random(), new Rotation2d(Math.random(), Math.random())),
                  new SwerveModulePosition(
                      Math.random(), new Rotation2d(Math.random(), Math.random())),
                  new SwerveModulePosition(
                      Math.random(), new Rotation2d(Math.random(), Math.random())),
                  new SwerveModulePosition(
                      Math.random(), new Rotation2d(Math.random(), Math.random()))
                },
                drive.getRawGyroRotation()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void configurePathPlanner() {}
}
