// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.subsystems.drive.DriveConstants.TunerConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the subsystem controlling the drivetrain/swerve base. Most of this code is taken
 * directly from the AdvantageKit swerve template and then further modified to fit our code
 * architecture.
 */
public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO; // Gyro
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR in that specific order
  private final SysIdRoutine sysId; // We didn't use this
  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro unavailable; heading accuracy is degraded.", AlertType.kError);
  private final Alert headingHeldAlert =
      new Alert("Gyro and wheel heading unavailable; holding last heading.", AlertType.kError);

  // Kinematics object helps translate general robot movement to individual swerve module movement
  // and vice versa
  private final SwerveDriveKinematics kinematics = DriveConstants.kSwerveKinematics;

  // Heading used by both odometry and field-relative driving. During a gyro outage it is estimated
  // from wheel travel; after recovery it includes an offset to keep that heading continuous.
  private Rotation2d rawGyroRotation;
  private Rotation2d gyroRecoveryOffset = Rotation2d.kZero;
  private SwerveModulePosition[] previousModulePositions;
  private boolean headingInitialized;
  private boolean gyroWasConnected;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    // Initialize the IO implementations based on passed parameters
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
    rawGyroRotation = Rotation2d.kZero;

    // Configure SysId (helps us figure out which feedforward values to use for the motors)
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Tells the robot that this current direction is zero
    // Requires us to face the robot perfectly forward on startup and then move once the robot code
    // is ready
    zeroYaw();
  }

  @Override
  public void periodic() {
    // Refresh the gyro and all four modules while preventing the odometry sampling thread from
    // modifying their queues. The hardware reads occur sequentially rather than simultaneously, so
    // this is one protected refresh batch. Always release the lock if an IO adapter throws.
    odometryLock.lock();
    try {
      // Modules are plain helper objects rather than registered subsystems, so Drive owns their
      // periodic input refresh.
      for (var module : modules) {
        module.periodic();
      }

      // The simulated gyro derives yaw from measured wheel travel. Refresh modules first so yaw
      // and wheel positions describe the same simulation step. REAL still reads the Pigeon here.
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
    } finally {
      odometryLock.unlock();
    }

    // Publish the freshly read drivetrain snapshot before commands execute. CommandScheduler calls
    // subsystem periodic methods before command execute methods, so commands now consume state from
    // this cycle instead of the previous cycle. This intentionally remains a simple 50 Hz path;
    // completing the unfinished high-frequency sample pipeline is separate follow-up work.
    SwerveModulePosition[] modulePositions = getModulePositions();
    updateHeading(modulePositions);
    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(Timer.getTimestamp(), modulePositions, rawGyroRotation));
    Logger.recordOutput("Drive/MeasuredPositions", modulePositions);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroWasConnected && Constants.kCurrentMode != Mode.SIM);
    Logger.recordOutput("Drive/Heading", rawGyroRotation);
  }

  /** Keep a continuous heading when gyro feedback is lost or returns with a different reference. */
  private void updateHeading(SwerveModulePosition[] modulePositions) {
    double wheelTurn = 0.0;
    boolean wheelsValid = hasValidWheelPositions();
    boolean wheelTurnValid = wheelsValid && previousModulePositions != null;
    if (wheelTurnValid) {
      SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];
      for (int i = 0; i < modules.length; i++) {
        deltas[i] =
            new SwerveModulePosition(
                modulePositions[i].distanceMeters - previousModulePositions[i].distanceMeters,
                modulePositions[i].angle);
      }
      wheelTurn = kinematics.toTwist2d(deltas).dtheta;
      wheelTurnValid = Double.isFinite(wheelTurn);
      if (!wheelTurnValid) {
        wheelTurn = 0.0;
      }
    }
    // Save a baseline every cycle, including healthy gyro cycles. The first sample establishes the
    // baseline so existing encoder distances at startup are not mistaken for fresh movement.
    // Discard the baseline after a failed read. On recovery, start fresh instead of treating
    // accumulated travel (or an encoder reset) across the outage as one new movement sample.
    previousModulePositions = wheelsValid ? modulePositions : null;
    boolean gyroConnected =
        gyroInputs.connected && Double.isFinite(gyroInputs.yawPosition.getRadians());
    if (gyroConnected) {
      if (headingInitialized && !gyroWasConnected) {
        // Include this cycle's wheel movement before anchoring the returning gyro. Keep the offset
        // on subsequent cycles: switching straight back to its raw yaw would cause a heading jump.
        gyroRecoveryOffset =
            rawGyroRotation.plus(new Rotation2d(wheelTurn)).minus(gyroInputs.yawPosition);
      }
      rawGyroRotation = gyroInputs.yawPosition.plus(gyroRecoveryOffset);
    } else {
      // Wheel travel is a temporary estimate, not an absolute heading reference. It can drift when
      // wheels slip and relies on usable module feedback; the gyro alert remains active in REAL.
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(wheelTurn));
    }
    boolean holdingHeading = !gyroConnected && !wheelTurnValid;
    headingHeldAlert.set(holdingHeading && Constants.kCurrentMode != Mode.SIM);
    Logger.recordOutput("Drive/UsingWheelHeading", !gyroConnected && wheelTurnValid);
    Logger.recordOutput("Drive/HoldingHeading", holdingHeading);
    gyroWasConnected = gyroConnected;
    headingInitialized = true;
  }

  private boolean hasValidWheelPositions() {
    for (Module module : modules) {
      if (!module.hasValidPosition()) {
        return false;
      }
    }
    return true;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Robot-relative speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates,
        TunerConstants
            .kSpeedAt12Volts); // Makes sure none of the target states are faster than what is
    // physically possible

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Runs all modules' drive motor at the specified output */
  public void runDriveOpenLoop(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runDriveOpenLoop(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Sets the sensor yaw and cached raw heading without changing the pose estimator.
   *
   * @param angle the requested sensor heading
   */
  public void setYaw(Rotation2d angle) {
    gyroIO.setYaw(angle);
    rawGyroRotation = angle;
    // An explicit reset replaces any recovery offset. During an outage, preserve the requested
    // heading and anchor the gyro to it when feedback returns.
    gyroRecoveryOffset = Rotation2d.kZero;
    previousModulePositions = hasValidWheelPositions() ? getModulePositions() : null;
  }

  /**
   * Creates the driver's heading-reset command, requiring the drivetrain and preserving X/Y.
   *
   * <p>REAL retains the existing estimator-zero and hardware-zero requests. SIM additionally aligns
   * the estimator with its immediately reset sensor. The physical gyro's offset and reset timing
   * need separate validation before adopting that alignment policy on REAL.
   */
  public Command resetHeading() {
    return runOnce(
        () -> {
          RobotState state = RobotState.getInstance();
          state.resetRotation(Rotation2d.kZero);
          setYaw(Rotation2d.kZero);
          if (Constants.kCurrentMode == Mode.SIM) {
            // Rebase the estimator's gyro offset and wheel baseline together. Otherwise the next
            // sample can restore the old heading offset after the sensor has already been zeroed.
            state.setPose(
                new Pose2d(state.getEstimatedPose().getTranslation(), Rotation2d.kZero),
                getModulePositions(),
                rawGyroRotation);
          }
        });
  }

  /** Zeros the gyro yaw. */
  public Command zeroYaw() {
    return Commands.runOnce(() -> this.setYaw(Rotation2d.kZero), this);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the continuous drive heading, including wheel fallback and gyro recovery offset. */
  public Rotation2d getRawGyroRotation() {
    return rawGyroRotation;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.kDriveBaseRadius;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
