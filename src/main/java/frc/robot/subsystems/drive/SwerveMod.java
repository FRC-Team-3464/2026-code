package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code SwerveMod} class contains/controls the io, inputs, and name of one swerve module.
 *
 * @author Maxwell Morgan
 */
public class SwerveMod {
  private final ModuleIO io;

  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleName name;

  public SwerveMod(ModuleIO io, ModuleName name) {
    this.io = io;
    this.name = name;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module/" + name.toString(), inputs);
  }

  /**
   * Runs the module to the desired state.
   *
   * @param desiredState Desired wheel speed and angle
   * @param isOpenLoop If true, uses percent output; if false, uses velocity closed-loop
   */
  public void runDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Cosine compensation reduces speed if wheel isn't pointing correctly
    double angleDiff = desiredState.angle.minus(getAngle()).getRadians();
    double compensatedSpeed = desiredState.speedMetersPerSecond * Math.cos(angleDiff);
    double speedRadPerSec = compensatedSpeed / DriveConstants.kWheelRadius;

    if (isOpenLoop) {
      // In meters per second
      double percentOutput = compensatedSpeed / DriveConstants.kPhysicalMaxSpeed;
      io.runDriveDutyCycle(percentOutput);
    } else {
      io.runDriveVelocity(speedRadPerSec);
    }

    // Turn control
    if (isTurnWithinDeadband(desiredState.angle)) {
      io.runTurnDutyCycle(0);
    } else {
      io.runTurnAngle(desiredState.angle);
    }
  }

  /** Stops all output to the module's motors. */
  public void stop() {
    io.runDriveDutyCycle(0);
    io.runTurnDutyCycle(0);
  }

  public void resetToAbsolute() {
    io.resetToAbsolute();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), inputs.data.turnPosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), inputs.data.turnPosition());
  }

  public Rotation2d getAngle() {
    return inputs.data.turnPosition();
  }

  public double getPositionMeters() {
    return inputs.data.drivePositionRad() * DriveConstants.kWheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.data.driveVelocityRadPerSec() * DriveConstants.kWheelRadius;
  }

  private boolean isTurnWithinDeadband(Rotation2d target) {
    return Math.abs(target.minus(getAngle()).getDegrees()) < 3;
  }

  public enum ModuleName {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    private final int index;

    ModuleName(int index) {
      this.index = index;
    }

    public int getIndex() {
      return this.index;
    }
  }
}
