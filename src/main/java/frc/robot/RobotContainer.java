// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DeviceIDs;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.commands.DriveCommands;
import frc.robot.control.Configurable;
import frc.robot.control.DefaultControls;
import frc.robot.control.DriverController;
import frc.robot.control.DriverControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.guts.Guts;
import frc.robot.subsystems.guts.Guts.GutSide;
import frc.robot.subsystems.guts.GutsIOSim;
import frc.robot.subsystems.guts.GutsIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.LedSection;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.CameraIOLimelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.List;

public class RobotContainer {
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final DriverController operator = new DriverController.XboxDriverController(1);

  private Drive drive;
  private Shooter leftShooter;
  private Shooter rightShooter;
  private Guts leftGuts;
  private Guts rightGuts;
  private Intake intake;
  private Vision vision;

  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // vision = new Vision(null, null);
        leftShooter =
            new Shooter(
                ShooterSide.LEFT,
                new HoodIOSparkMax(ShooterSide.LEFT),
                new FlywheelIOTalonFX(ShooterSide.LEFT));
        rightShooter =
            new Shooter(
                ShooterSide.RIGHT,
                new HoodIOSparkMax(ShooterSide.RIGHT),
                new FlywheelIOTalonFX(ShooterSide.RIGHT));
        leftGuts = new Guts(GutSide.LEFT, new GutsIOSparkMax(DeviceIDs.kLeftGuts));
        rightGuts = new Guts(GutSide.RIGHT, new GutsIOSparkMax(DeviceIDs.kRightGuts));
        intake = new Intake(new IntakeIOHardware());
        vision =
            new Vision(
                new VisionConsumer() {
                  @Override
                  public void accept(
                      Pose2d visionRobotPoseMeters,
                      double timestampSeconds,
                      Matrix<N3, N1> visionMeasurementStdDevs) {
                    RobotState.getInstance()
                        .addVisionMeasurement(
                            new VisionMeasurement(
                                timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs));
                  }
                },
                new CameraIOLimelight(
                    "limelight-front", () -> RobotState.getInstance().getRotation()),
                new CameraIOLimelight(
                    "limelight-left", () -> RobotState.getInstance().getRotation()));
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        leftShooter =
            new Shooter(ShooterSide.LEFT, new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
        rightShooter =
            new Shooter(ShooterSide.RIGHT, new TurretIOSim(), new HoodIOSim(), new FlywheelIOSim());
        // vision = new Vision(null, null);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // vision = new Vision(null, null);
        leftGuts = new Guts(GutSide.LEFT, new GutsIOSim());
        rightGuts = new Guts(GutSide.RIGHT, new GutsIOSim());
        intake = new Intake(new IntakeIOSim());

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
        leftShooter = new Shooter(null, null, null, null);
        rightShooter = new Shooter(null, null, null, null);
        // vision = new Vision(null, new CameraIO[] {});
        break;
    }
    Leds.getInstance().solid(LedSection.ALL, Color.kCyan);

    configureBindings();
  }

  private void configureBindings() {
    List.<Configurable>of(
            new DefaultControls(driver, operator, drive, leftShooter, rightShooter),
            new DriverControls(
                driver, operator, drive, leftShooter, rightShooter, leftGuts, rightGuts, intake)
            // // TODO: Implement ZoneControls
            // // ,new ZoneControls()
            )
        .forEach(Configurable::configure);
  }

  public void robotPeriodic() {
    OdometryObservation obs =
        new OdometryObservation(
            Timer.getTimestamp(), drive.getModulePositions(), drive.getRawGyroRotation());
    RobotState.getInstance().addOdometryObservation(obs);
    RobotState.getInstance().setRobotVelocity(drive.getChassisSpeeds());
    // System.out.println(RobotState.getInstance().getEstimatedPose().getX());
    // System.out.println(RobotState.getInstance().getRobotVelocity().vxMetersPerSecond);
    // System.out.println(driver.getLeftX());
  }

  public Command getAutonomousCommand() {
    return DriveCommands.feedforwardCharacterization(drive);
    // return Commands.print("No autonomous command configured");
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
        "Shoot Both At Hub",
        Shooter.shootBothAtTargetNoTurret(
                leftShooter,
                rightShooter,
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()))
            .alongWith(leftGuts.runGutForward(), rightGuts.runGutForward()));

    NamedCommands.registerCommand("Intake/Index Fuel", intake.intake());
    NamedCommands.registerCommand("Deploy Intake", intake.intake());
    NamedCommands.registerCommand("Retract Intake", intake.intake());

    NamedCommands.registerCommand(
        "Turn Robot To Hub",
        DriveCommands.turnToPoint(
            drive,
            () -> RobotState.getInstance().getEstimatedPose(),
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d())));
  }
}
