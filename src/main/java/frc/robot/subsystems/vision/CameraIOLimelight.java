// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** {@code CameraIO} implementation for running on a real Limelight camera. */
public class CameraIOLimelight implements CameraIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new CameraIOLimelight.
   *
   * @param name The configured name of the Limelight camera.
   * @param rotationSupplier Supplier for the current estimated rotation.
   */
  public CameraIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    this.megatag1Subscriber =
        table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    this.megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      String rejection = validatePoseMessage(rawSample.value);
      if (!rejection.isEmpty()) {
        // Keep a count and the latest reason in the camera logs, even after valid readings resume,
        // so intermittent faults can be diagnosed. Skip only this message and keep processing.
        inputs.rejectedPoseMessages++;
        inputs.lastPoseRejection = "MEGATAG1: " + rejection;
        continue;
      }
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      String rejection = validatePoseMessage(rawSample.value);
      if (!rejection.isEmpty()) {
        // Keep a count and the latest reason in the camera logs, even after valid readings resume,
        // so intermittent faults can be diagnosed. Skip only this message and keep processing.
        inputs.rejectedPoseMessages++;
        inputs.lastPoseRejection = "MEGATAG2: " + rejection;
        continue;
      }
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Checks the message before any pose or tag data is read. Limelight's header has 11 values; newer
   * firmware appends seven values per detected tag. Keep accepting the older header-only format,
   * but never treat a partially received tag block as a complete observation.
   *
   * <p>This validates message contents, not camera accuracy or frame freshness. Vision still owns
   * field-boundary and ambiguity filtering; the estimator still owns measurement weighting.
   *
   * @return an empty string for a valid message, otherwise a reason suitable for the camera log
   */
  private static String validatePoseMessage(double[] values) {
    if (values.length < 11) return "Incomplete pose header";
    for (double value : values) {
      if (!Double.isFinite(value)) return "Non-finite pose or tag value";
    }
    double tagCount = values[7];
    if (tagCount < 0 || tagCount > Integer.MAX_VALUE || tagCount != Math.rint(tagCount)) {
      return "Invalid tag count";
    }
    if (values.length != 11 && (values.length - 11L != 7L * (long) tagCount)) {
      return "Tag data does not match tag count";
    }
    if (values[6] < 0) return "Negative capture latency";
    // A detected tag must have a positive distance. Zero would give it zero uncertainty and,
    // for MegaTag 2, could turn the intentionally infinite heading uncertainty into NaN.
    if (values[9] < 0 || (tagCount > 0 && values[9] == 0)) {
      return "Invalid average tag distance";
    }
    for (int i = 11; i < values.length; i += 7) {
      double id = values[i];
      if (id < 0 || id > Integer.MAX_VALUE || id != Math.rint(id)) {
        return "Invalid tag ID";
      }
      if (values[i + 4] < 0 || values[i + 5] < 0) return "Negative tag distance";
      if (values[i + 6] < 0 || values[i + 6] > 1) return "Invalid tag ambiguity";
    }
    return "";
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  public static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
