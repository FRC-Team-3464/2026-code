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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.control.Configurable;
import frc.robot.control.DefaultControls;
import frc.robot.control.DriverController;
import frc.robot.control.DriverControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.util.GeomUtil;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final DriverController operator = new DriverController.XboxDriverController(1);

  private Drive drive;
  private Indexer indexer;
  private Intake intake;
  private Shooter shooter;
  private Vision vision;

  private static Field2d field2d = new Field2d();
  private static Field2d targetField2d = new Field2d();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    Supplier<Rotation2d> robotRotationSupplier = () -> RobotState.getInstance().getRotation();

    SmartDashboard.putData("FieldInstance", field2d);
    SmartDashboard.putData("TargetField", targetField2d);
    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());
    switch (Constants.kCurrentMode) {
      case REAL -> {
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
        indexer = new Indexer(new IndexerIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new TurretIOSparkMax() {
        }, new HoodIOSparkMax(), new FlywheelIOTalonFX());
        vision = new Vision(
            new VisionConsumer() {
              public void accept(
                  Pose2d visionRobotPoseMeters,
                  double timestampSeconds,
                  edu.wpi.first.math.Matrix<N3, N1> visionMeasurementStdDevs) {

                RobotState.getInstance()
                    .addVisionMeasurement(
                        new VisionMeasurement(
                            timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
              };
            },
            new CameraIOLimelight("limelight-front", robotRotationSupplier),
            new CameraIOLimelight("limelight-one", robotRotationSupplier));
      }
      case SIM -> {
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
      }
    }

    configureBindings();

    if (Constants.kCurrentMode == Mode.REAL) {
      configurePathPlanner();

      autoChooser = AutoBuilder.buildAutoChooser();

      SmartDashboard.putData(autoChooser);
    }
  }

  private void configureBindings() {
    List.<Configurable>of(
        new DefaultControls(driver, operator, drive, indexer, intake, shooter),
        new DriverControls(driver, operator, drive, shooter, intake, indexer))
        .forEach(Configurable::configure);
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
    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());
  }

  public Command getAutonomousCommand() {
    if (Constants.kCurrentMode == Mode.REAL) {
      return autoChooser.getSelected();
    }
    return Commands.print("No autonomous command configured");
  }

  public void configurePathPlanner() {
    NamedCommands.registerCommand("Shoot", shooter.shootAtTargetNoRotation(() -> RobotState.getInstance().getTurretTarget()));
    NamedCommands.registerCommand("Index", indexer.index());
    NamedCommands.registerCommand("Intake", intake.intake());
    NamedCommands.registerCommand("DeployIntake", intake.deployOpenLoop().withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("RetractIntake", intake.retractOpenLoop().withTimeout(Seconds.of(2)));
   }
}
