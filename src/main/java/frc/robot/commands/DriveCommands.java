package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class DriveCommands {

  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier isRobotRelative) {
    return Commands.run(
        () -> {
          double x = xSupplier.getAsDouble();
          double y = ySupplier.getAsDouble();
          double rot = rotationSupplier.getAsDouble();

          x = Math.abs(x) > DriveConstants.kDeadband ? x * Math.abs(x) : 0.0;
          y = Math.abs(y) > DriveConstants.kDeadband ? y * Math.abs(y) : 0.0;
          rot = Math.abs(rot) > DriveConstants.kDeadband ? rot * Math.abs(rot) : 0.0;

          x *= DriveConstants.kPhysicalMaxSpeed;
          y *= DriveConstants.kPhysicalMaxSpeed;
          rot *= DriveConstants.kMaxTeleAngularSpeed;

          ChassisSpeeds speeds =
              isRobotRelative.getAsBoolean()
                  ? new ChassisSpeeds(x, y, rot)
                  : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, drive.getRawGyroRotation());

          drive.runVelocity(speeds, false);
        },
        drive);
  }
}
