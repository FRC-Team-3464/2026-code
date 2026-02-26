package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.Supplier;

/**
 * Represents a 2D zone on the field. Zones can be primitive (circle, rectangle,
 * polygon) or composite (union, intersection, difference, complement).
 *
 * Inspired by Team 4481's zone system.
 *
 * Example usage:
 * <pre><p>
 *   Zone trenchZone = new RectangleZone(new Translation2d(1, 1), new Translation2d(4, 3));
 *   Zone safeZone   = new CircleZone(new Translation2d(5, 5), 1.5);
 *   Zone combined   = trenchZone.union(safeZone);
 *   combined.contains(robot::getPose).onTrue(hood.down());
 */
public interface Zone {

  /**
   * Returns a Trigger that is active when the supplied translation is inside this zone.
   *
   * @param translation a Supplier providing the current Translation2d to check
   * @return a Trigger that polls containment
   */
  Trigger contains(Supplier<Translation2d> translation);

  /** Returns a zone that is the union (A ∪ B) of this zone and another. */
  default Zone union(Zone other) {
    return translation -> this.contains(translation).or(other.contains(translation));
  }

  /** Returns a zone that is the intersection (A ∩ B) of this zone and another. */
  default Zone intersection(Zone other) {
    return translation -> this.contains(translation).and(other.contains(translation));
  }

  /**
   * Returns a zone representing the difference (A \ B): points in this zone that are NOT in the
   * other zone.
   */
  default Zone difference(Zone other) {
    return translation -> this.contains(translation).and(other.contains(translation).negate());
  }

  /** Returns the complement of this zone (points NOT in this zone). */
  default Zone complement() {
    return translation -> this.contains(translation).negate();
  }

  /**
   * A circular zone defined by a center point and a radius. A translation is inside if its distance
   * to the center is less than the radius.
   */
  class CircleZone implements Zone {
    private final Translation2d center;
    private final double radius;

    public CircleZone(Translation2d center, double radius) {
      this.center = center;
      this.radius = radius;
    }

    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
      return new Trigger(() -> translation.get().getDistance(center) < radius);
    }
  }

  /**
   * An axis-aligned rectangular zone defined by two corner points. A translation is inside if its x
   * and y coordinates fall within the bounding box.
   */
  class RectangleZone implements Zone {
    private final double minX, maxX, minY, maxY;

    public RectangleZone(Translation2d cornerA, Translation2d cornerB) {
      this.minX = Math.min(cornerA.getX(), cornerB.getX());
      this.maxX = Math.max(cornerA.getX(), cornerB.getX());
      this.minY = Math.min(cornerA.getY(), cornerB.getY());
      this.maxY = Math.max(cornerA.getY(), cornerB.getY());
    }

    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
      return new Trigger(
          () -> {
            Translation2d t = translation.get();
            return t.getX() >= minX && t.getX() <= maxX && t.getY() >= minY && t.getY() <= maxY;
          });
    }
  }

  /**
   * A polygonal zone defined by an ordered list of vertices.
   *
   * <p>Uses a cross-product (winding) approach: for each edge of the polygon, the point must be on
   * the same side (left side for CCW winding). Works correctly for convex polygons. For concave
   * polygons, use a ray-casting approach instead.
   */
  class PolygonZone implements Zone {
    private final List<Translation2d> vertices;

    /**
     * @param vertices ordered vertices of the polygon (CCW winding for correct results)
     */
    public PolygonZone(List<Translation2d> vertices) {
      if (vertices.size() < 3) {
        throw new IllegalArgumentException("A polygon must have at least 3 vertices.");
      }
      this.vertices = List.copyOf(vertices);
    }

    @Override
    public Trigger contains(Supplier<Translation2d> translation) {
      return new Trigger(() -> isInsidePolygon(translation.get()));
    }

    /**
     * Ray-casting algorithm for point-in-polygon detection. Works for both convex and concave
     * (simple) polygons.
     */
    private boolean isInsidePolygon(Translation2d point) {
      int n = vertices.size();
      boolean inside = false;
      double px = point.getX();
      double py = point.getY();

      for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = vertices.get(i).getX(), yi = vertices.get(i).getY();
        double xj = vertices.get(j).getX(), yj = vertices.get(j).getY();

        boolean intersects =
            ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
        if (intersects) inside = !inside;
      }
      return inside;
    }
  }
}
