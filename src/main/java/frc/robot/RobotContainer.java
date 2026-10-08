// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.wiring.RealRobotWiring;
import frc.robot.wiring.RobotWiring;
import frc.robot.wiring.SimRobotWiring;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

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

  // Published as "Autonomous" on SmartDashboard so the team can select an auto before a match.
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Map<Command, Pose2d> autoStartingPoses = new HashMap<>();

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

    configurePathPlanner();
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
    Pose2d start = getSelectedAutoStartingPose();
    field2d.getObject("Auto Start").setPoses(start == null ? List.of() : List.of(start));
    SmartDashboard.putBoolean("Auto Start/Available", start != null);
    SmartDashboard.putString(
        "Auto Start/Alliance",
        DriverStation.getAlliance().map(Enum::name).orElse("Unknown - blue preview only"));
    SmartDashboard.putString(
        "Auto Start/Placement",
        start == null
            ? "Select a URI auto"
            : String.format(
                java.util.Locale.ROOT,
                "Robot center: X %.3f m, Y %.3f m; chassis heading %.1f deg; turret centered rearward",
                start.getX(),
                start.getY(),
                start.getRotation().getDegrees()));
  }

  /** Uses the same blue-origin pose and alliance flip as PathPlanner's autonomous reset. */
  private Pose2d getSelectedAutoStartingPose() {
    Pose2d blueStart = autoStartingPoses.get(autoChooser.getSelected());
    return blueStart == null
        ? null
        : AllianceFlipUtil.shouldFlip() ? FlippingUtil.flipFieldPose(blueStart) : blueStart;
  }

  /**
   * Explicit setup action for both REAL and SIM; it changes the estimate, not physical position.
   */
  private void applySelectedAutoStartingPose() {
    // Never let a dashboard click teleport the estimate while driving or running an auto.
    if (!DriverStation.isDisabled()) {
      return;
    }
    Pose2d start = getSelectedAutoStartingPose();
    if (start == null || DriverStation.getAlliance().isEmpty()) {
      DriverStation.reportWarning(
          "Select a URI auto and set the alliance before applying its start.", false);
      return;
    }
    RobotState.getInstance().setPose(start, drive.getModulePositions(), drive.getRawGyroRotation());
  }

  /** Returns the dashboard-selected autonomous routine, or Do Nothing when none is selected. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Configures PathPlanner and publishes the three 2026 URI autos. */
  private void configurePathPlanner() {
    // Startup must remain safe if a deployed PathPlanner file is missing or invalid.
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Autonomous", autoChooser);
    SmartDashboard.putData(
        "Apply Auto Starting Pose",
        Commands.runOnce(this::applySelectedAutoStartingPose).ignoringDisable(true));

    // Register actions before loading .auto files that refer to them.
    NamedCommands.registerCommand(
        "Shoot",
        shooter.trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne(
            () -> RobotState.getInstance().getShooterTarget()));
    // The URI autos request Index inside a bounded shooting window. Pause feeding whenever the
    // turret, hood, or flywheel loses readiness and resume only if all three become ready again.
    NamedCommands.registerCommand("Index", indexer.indexWhileReady(shooter::readyToShoot));
    NamedCommands.registerCommand("Intake", intake.intake());
    NamedCommands.registerCommand(
        "DeployIntake", intake.deployOpenLoop().withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand(
        "RetractIntake", intake.retractOpenLoop().withTimeout(Seconds.of(2)));
    // Some stored autos contain a future climb action. This robot has no climber, so resolving the
    // name to a no-op cannot move hardware. Replace this only when a real climber exists.
    NamedCommands.registerCommand("climb", Commands.none());

    try {
      // Keep the runtime robot model aligned with the PathPlanner editor settings. The 2026
      // values and controller gains still need physical robot validation.
      RobotConfig config = RobotConfig.fromGUISettings();
      // Match the field dimensions used by the hub target and alliance utilities.
      FlippingUtil.fieldSizeX = FieldConstants.fieldLength;
      FlippingUtil.fieldSizeY = FieldConstants.fieldWidth;
      AutoBuilder.configure(
          () -> RobotState.getInstance().getEstimatedPose(),
          pose ->
              RobotState.getInstance()
                  .setPose(pose, drive.getModulePositions(), drive.getRawGyroRotation()),
          drive::getChassisSpeeds,
          (speeds, feedforwards) -> drive.runVelocity(speeds),
          new PPHolonomicDriveController(new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0)),
          config,
          AllianceFlipUtil::shouldFlip,
          drive);
      // Other stored autos can intake after a missed shot. With no sensor confirming that fuel
      // exited, keep those routes unavailable until continuation is known to be safe.
      addAutoOption("URI Center");
      addAutoOption("URI Left Depot");
      addAutoOption("URI Right Outpost");
    } catch (IOException | ParseException | RuntimeException e) {
      DriverStation.reportError(
          "PathPlanner configuration failed: " + e.getMessage(), e.getStackTrace());
    }
  }

  /** Loads one auto without preventing the other chooser options from loading. */
  private void addAutoOption(String name) {
    try {
      // A missing or malformed file can be replaced with a no-op internally. Do not offer it as a
      // runnable option unless PathPlanner can find a nonempty path group.
      if (PathPlannerAuto.getPathGroupFromAutoFile(name).isEmpty()) {
        throw new IllegalStateException("routine contains no paths");
      }
      PathPlannerAuto auto = new PathPlannerAuto(name);
      autoStartingPoses.put(auto, auto.getStartingPose());
      autoChooser.addOption(name, auto);
    } catch (IOException | ParseException | RuntimeException e) {
      DriverStation.reportError(
          "Cannot load autonomous routine '" + name + "': " + e.getMessage(), e.getStackTrace());
    }
  }
}
