package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Enum representing common compass directions (e.g., North, Northeast, East). */
public enum Direction {
  NORTH(0),
  NORTHEAST(45),
  EAST(90),
  SOUTHEAST(135),
  SOUTH(180),
  SOUTHWEST(225),
  WEST(270),
  NORTHWEST(315);

  private final double angleDegrees;

  Direction(double angleDegrees) {
    this.angleDegrees = angleDegrees;
  }

  public double getAngleDegrees() {
    return angleDegrees;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(angleDegrees);
  }

  public Direction rotateRight45() {
    return values()[(this.ordinal() + 1) % 8];
  }

  public Direction rotateLeft45() {
    return values()[(this.ordinal() + 7) % 8];
  }

  public Direction rotateRight90() {
    return values()[(this.ordinal() + 2) % 8];
  }

  public Direction rotateLeft90() {
    return values()[(this.ordinal() + 6) % 8];
  }

  public Direction opposite() {
    return values()[(this.ordinal() + 4) % 8];
  }

  public int getDx() {
    switch (this) {
      case EAST:
      case NORTHEAST:
      case SOUTHEAST:
        return 1;
      case WEST:
      case NORTHWEST:
      case SOUTHWEST:
        return -1;
      default:
        return 0;
    }
  }

  public int getDy() {
    switch (this) {
      case NORTH:
      case NORTHEAST:
      case NORTHWEST:
        return 1;
      case SOUTH:
      case SOUTHEAST:
      case SOUTHWEST:
        return -1;
      default:
        return 0;
    }
  }

  public boolean isCardinal() {
    return this == NORTH || this == SOUTH || this == EAST || this == WEST;
  }

  public boolean isDiagonal() {
    return !isCardinal();
  }

  public static Direction fromAngle(double angleDegrees) {
    angleDegrees = ((angleDegrees % 360) + 360) % 360; // normalize 0–359
    int index = (int) Math.round(angleDegrees / 45.0) % 8;
    return values()[index];
  }

  @Override
  public String toString() {
    return name().charAt(0) + name().substring(1).toLowerCase();
  }
}
