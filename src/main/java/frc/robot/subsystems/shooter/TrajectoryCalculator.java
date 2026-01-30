package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryCalculator {

  public record ShooterParams(double wheelRPM, double hoodAngle, Rotation2d turretAngle) {}
}
