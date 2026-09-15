// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.subsystems.leds.Leds;
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
import java.util.List;
import java.util.function.Supplier;

public class RobotContainer {
  // Declare and initialize both controllers
  // Uses our custom class to make switching controllers based on driver preference easy
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final DriverController operator = new DriverController.XboxDriverController(1);

  // Declare all subsystems (to be initialized in constructor)
  private Drive drive;
  private Indexer indexer;
  private Intake intake;
  private Shooter shooter;
  private Leds leds;
  private Vision vision;

  // These are used for visualization in SmartDashboard (not necessary for controlling robot)
  private static Field2d field2d = new Field2d();
  private static Field2d targetField2d = new Field2d();

  // Allows us to use SmartDashboard to choose an auto path (we didn't use it this year)
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // This is a supplier that will return the current rotation of the robot relative to the field
    // Call it with robotRotationSupplier.get()
    // It uses the pose from the RobotState class
    Supplier<Rotation2d> robotRotationSupplier = () -> RobotState.getInstance().getRotation();

    // Put the robot graphics on SmartDashboard (not necessary for controlling robot)
    SmartDashboard.putData("FieldInstance", field2d);
    SmartDashboard.putData("TargetField", targetField2d);
    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());

    // Changes the way that the subsystems are initialized based on if we're running the real robot
    // or a simulation
    // If real -> use the real hardware io implementations, if sim -> use the sim io implementations
    switch (Constants.kCurrentMode) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        indexer = new Indexer(new IndexerIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        leds = Leds.getInstance();
        shooter =
            new Shooter(new TurretIOSparkMax() {}, new HoodIOSparkMax(), new FlywheelIOTalonFX());
        vision =
            new Vision(
                new VisionConsumer() {
                  // We have to create an implementation of the accept function to tell the Vision
                  // subsystem what to do with its measurements
                  public void accept(
                      Pose2d visionRobotPoseMeters,
                      double timestampSeconds,
                      edu.wpi.first.math.Matrix<N3, N1> visionMeasurementStdDevs) {

                    // Just send them to the RobotState class
                    RobotState.getInstance()
                        .addVisionMeasurement(
                            new VisionMeasurement(
                                timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
                  }
                  ;
                },
                new CameraIOLimelight("limelight-front", robotRotationSupplier),
                new CameraIOLimelight("limelight-one", robotRotationSupplier));
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        indexer = new Indexer(new IndexerIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
      }
    }

    // Configures the driver controls
    configureBindings();

    // We would use this for PathPlanner autos, but we didn't have time to try it this season
    // if (Constants.kCurrentMode == Mode.REAL) {
    // configurePathPlanner();

    // // autoChooser = AutoBuilder.buildAutoChooser();

    // // SmartDashboard.putData(autoChooser);
    // }
  }

  /** Binds robot actions to operator and driver controls. */
  private void configureBindings() {
    List.<Configurable>of(
            new DefaultControls(driver, operator, drive, indexer, intake, shooter),
            new DriverControls(driver, operator, drive, shooter, intake, indexer))
        .forEach(Configurable::configure);
  }

  /** This is called every 20ms. */
  public void robotPeriodic() {
    // Gets the current measured robot heading (rotation) from the drive subsystem and sends it to
    // the RobotState class
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

    // Update the SmartDashboard visualizations
    targetField2d.setRobotPose(GeomUtil.toPose2d(RobotState.getInstance().getShooterTarget()));
    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());
  }

  public Command getAutonomousCommand() {
    // Simple manual command that makes the robot aim at the hub and then shoots the fuel
    return Commands.parallel(
        shooter.trackTargetFlywheel(() -> RobotState.getInstance().getShooterTarget()),
        shooter.trackTargetHood(() -> RobotState.getInstance().getShooterTarget()),
        Commands.sequence(
            Commands.waitUntil(shooter::flywheelAtGoal),
            indexer.index())); // Don't start shooting until we're done aiming
  }

  public void configurePathPlanner() {
    // Basically just builds the PathPlanner configuration
    // RobotConfig config;
    // try {
    // config = RobotConfig.fromGUISettings();

    // AutoBuilder.configure(
    // () -> RobotState.getInstance().getEstimatedPose(),
    // (Pose2d pose) -> RobotState.getInstance().setPose(pose),
    // () -> RobotState.getInstance().getRobotVelocity(),
    // (speeds, feedforwards) -> drive.runVelocity(speeds),
    // new PPHolonomicDriveController(new PIDConstants(5.0, 0, 0), new
    // PIDConstants(5.0, 0,
    // 0)),
    // config,
    // AllianceFlipUtil::shouldFlip,
    // drive);

    // } catch (Exception e) {
    // e.printStackTrace();
    // }

    // Add the robot actions to PathPlanner so we can actually put them in the paths
    NamedCommands.registerCommand(
        "Shoot",
        shooter.shootAtTargetNoRotation(() -> RobotState.getInstance().getShooterTarget()));
    NamedCommands.registerCommand("Index", indexer.index());
    NamedCommands.registerCommand("Intake", intake.intake());
    NamedCommands.registerCommand(
        "DeployIntake", intake.deployOpenLoop().withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand(
        "RetractIntake", intake.retractOpenLoop().withTimeout(Seconds.of(2)));
  }
}
