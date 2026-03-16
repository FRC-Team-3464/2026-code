// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.control.DriverController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.util.GeomUtil;
import java.util.function.Supplier;

public class RobotContainer {
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final DriverController operator = new DriverController.XboxDriverController(1);

  private Vision vision;
  private Drive drive;

  public static Field2d field2d = new Field2d();
  public static Field2d targetField2d = new Field2d();

  public RobotContainer() {

    Supplier<Rotation2d> robotRotationSupplier = () -> RobotState.getInstance().getRotation();

    SmartDashboard.putData("FieldInstance", field2d);
    SmartDashboard.putData("TargetField", targetField2d);
    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());
    switch (Constants.kCurrentMode) {
      case REAL:
        
      case SIM:

      case REPLAY:
      default:
        
    }
    vision =
        new Vision(
            new VisionConsumer() {
              public void accept(
                  Pose2d visionRobotPoseMeters,
                  double timestampSeconds,
                  edu.wpi.first.math.Matrix<N3, N1> visionMeasurementStdDevs) {

                RobotState.getInstance()
                    .addVisionMeasurement(
                        new VisionMeasurement(
                            timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
              }
              ;
            },
            new CameraIOLimelight("limelight-front", robotRotationSupplier),
            new CameraIOLimelight("limelight-one", robotRotationSupplier));
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
  }

  public void robotPeriodic() {
    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                Timer.getTimestamp(),
                new SwerveModulePosition[] {
                  new SwerveModulePosition(),
                  new SwerveModulePosition(),
                  new SwerveModulePosition(),
                  new SwerveModulePosition()
                },
                drive.getRawGyroRotation()));

    targetField2d.setRobotPose(GeomUtil.toPose2d(RobotState.getInstance().getTurretTarget()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void configurePathPlanner() {}
}
