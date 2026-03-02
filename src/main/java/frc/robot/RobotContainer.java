// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState.OdometryObservation;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSide;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.List;

public class RobotContainer {
  private final DriverController driver = new DriverController.XboxDriverController(0);
  private final Joystick operator = new Joystick(Constants.kOperatorControllerPort);

  private Drive drive;
  private Shooter leftShooter;
  private Shooter rightShooter;
  private Guts leftGuts;
  private Guts rightGuts;
  private Intake intake;
  // private Vision vision;

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
        // leftShooter =
        //     new Shooter(
        //         ShooterSide.LEFT,
        //         new TurretIOSparkMax(DeviceIDs.kLeftTurretAzimuth),
        //         new HoodIOSparkMax(DeviceIDs.kLeftTurretHood),
        //         new FlywheelIOTalonFX(DeviceIDs.kLeftTurretFlywheel));
        // rightShooter =
        //     new Shooter(
        //         ShooterSide.RIGHT,
        //         new TurretIOSparkMax(DeviceIDs.kRightTurretAzimuth),
        //         new HoodIOSparkMax(DeviceIDs.kRightTurretHood),
        //         new FlywheelIOTalonFX(DeviceIDs.kRightTurretFlywheel));
        // leftGuts = new Guts(GutSide.LEFT, new GutsIOSparkMax(DeviceIDs.kLeftGuts));
        // rightGuts = new Guts(GutSide.RIGHT, new GutsIOSparkMax(DeviceIDs.kRightGuts));
        // intake = new Intake(new IntakeIOHardware());
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

    // if (Constants.kCurrentMode == Constants.Mode.REAL) {
    // try {
    // Constants.kRobotConfig = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    // // Handle exception as needed
    // e.printStackTrace();
    // }
    // }

    // configurePathPlanner();
    configureBindings();
  }

  private void configureBindings() {
    List.<Configurable>of(
            new DefaultControls(driver, operator, drive, leftShooter, rightShooter),
            new DriverControls(
                driver, operator, drive, leftShooter, rightShooter, leftGuts, rightGuts, intake)
            //         // TODO: Implement ZoneControls
            //         // ,new ZoneControls()
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
