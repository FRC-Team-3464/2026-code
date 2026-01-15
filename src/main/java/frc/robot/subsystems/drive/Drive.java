package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final SwerveMod[] modules;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  @AutoLogOutput(key = "Drive/WantedStates")
  private SwerveModuleState[] wantedStates;

  @AutoLogOutput(key = "Drive/ActualStates")
  private SwerveModuleState[] actualStates;

  private SwerveModulePosition[] lastModulePositions;

  private Rotation2d rawGyroRotation = new Rotation2d();
  static final Lock m_odometryLock = new ReentrantLock();

  public Drive(SwerveMod[] modules, GyroIO gyroIO) {
    this.modules = modules;
    this.gyroIO = gyroIO;

    wantedStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    actualStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
  }

  @Override
  public void periodic() {
    m_odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (SwerveMod mod : modules) {
      mod.periodic();
    }
    actualStates = getModuleStates();
    m_odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (SwerveMod mod : modules) {
        mod.stop();
      }
    }

    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      SwerveModulePosition current = modules[i].getPosition();
      moduleDeltas[i] =
          new SwerveModulePosition(
              current.distanceMeters - lastModulePositions[i].distanceMeters, current.angle);
      lastModulePositions[i] = current;
    }

    if (gyroInputs.data.connected()) {
      rawGyroRotation = gyroInputs.data.yawPosition();
    } else {
      Twist2d twist = DriveConstants.swerveKinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
  }

  public void runVelocity(ChassisSpeeds speeds, boolean isOpenLoop) {
    // ChassisSpeeds newSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var states = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTeleDriveSpeed);
    for (int i = 0; i < 4; i++) {
      states[i].optimize(modules[i].getAngle());
      wantedStates[i] = states[i];
      modules[i].runDesiredState(states[i], isOpenLoop);
    }
  }

  /** Stops all output to the modules' motors. */
  public void stopModules() {
    for (SwerveMod mod : modules) {
      mod.stop();
    }
  }

  public double getYawVelocity() {
    return this.gyroInputs.data.yawVelocityRadPerSec();
  }

  /**
   * Retrieves the current position of all swerve modules.
   *
   * @return An array of {@link SwerveModulePosition} objects, one for each sweve module, ordered
   *     according to the module array in {@code modules}.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Retrieves the current state of all swerve modules.
   *
   * @return An array of {@link SwerveModuleState} objects, one for each sweve module, ordered
   *     according to the module array in {@code modules}.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    return moduleStates;
  }

  public Rotation2d getRawGyroRotation() {
    return this.rawGyroRotation;
  }
}
