// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.control.Configurable;
import frc.robot.control.DefaultControls;
import frc.robot.control.DriverController;
import frc.robot.control.DriverControllerFactory;
import frc.robot.control.DriverControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.GeomUtil;
import frc.robot.wiring.RealRobotWiring;
import frc.robot.wiring.RobotWiring;
import frc.robot.wiring.SimRobotWiring;
import java.util.List;
import java.util.function.Supplier;

public class RobotContainer {
  // Driver and operator may use different controller layouts without changing their bindings.
  private final DriverController driver =
      DriverControllerFactory.create(
          Constants.kCurrentMode,
          Constants.kDriverControllerProfile,
          Constants.kDriverControllerPort);
  private final DriverController operator =
      DriverControllerFactory.create(
          Constants.kCurrentMode,
          Constants.kOperatorControllerProfile,
          Constants.kOperatorControllerPort);

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

    // Select the hardware family once. The subsystem construction below is shared so commands and
    // subsystem behavior cannot accidentally drift between the real robot and simulation.
    RobotWiring wiring =
        switch (Constants.kCurrentMode) {
          case REAL -> new RealRobotWiring();
          case SIM -> new SimRobotWiring(() -> drive.getModulePositions());
          case REPLAY ->
              throw new IllegalStateException(
                  "REPLAY mode must be rejected before RobotContainer is constructed.");
        };

    drive =
        new Drive(
            wiring.createGyro(),
            wiring.createFrontLeftModule(),
            wiring.createFrontRightModule(),
            wiring.createBackLeftModule(),
            wiring.createBackRightModule());
    indexer = new Indexer(wiring.createIndexer());
    intake = new Intake(wiring.createIntake());

    leds = new Leds(wiring.createLeds());

    shooter = new Shooter(wiring.createTurret(), wiring.createHood(), wiring.createFlywheel());

    CameraIO[] cameras = wiring.createCameras(robotRotationSupplier);
    if (cameras.length > 0) {
      vision = new Vision(this::acceptVisionMeasurement, cameras);
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

  /** Sends an accepted camera measurement to the shared robot pose estimator. */
  private void acceptVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    RobotState.getInstance()
        .addVisionMeasurement(
            new VisionMeasurement(
                timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
  }

  /** Publishes the current target and estimated robot pose after the scheduler updates state. */
  public void updateDashboard() {
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
