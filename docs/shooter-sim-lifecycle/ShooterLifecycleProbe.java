package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Temporary SIM-only counters for the shooter lifecycle diagnostic. Remove after recording results.
 */
public final class ShooterLifecycleProbe {
  public enum Event {
    ROBOT_CYCLE,
    HOOD_PERIODIC,
    HOOD_IO_UPDATE,
    HOOD_SIM_STEP,
    TURRET_PERIODIC,
    TURRET_IO_UPDATE,
    TURRET_SIM_STEP,
    TURRET_POST_SCHEDULER,
    FLYWHEEL_PERIODIC,
    FLYWHEEL_IO_UPDATE,
    FLYWHEEL_SIM_STEP
  }

  private static final long[] counts = new long[Event.values().length];
  private static Command trackingCommand;

  private ShooterLifecycleProbe() {}

  public static void record(Event event) {
    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      counts[event.ordinal()]++;
    }
  }

  public static void setTrackingCommand(Command command) {
    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      trackingCommand = command;
    }
  }

  public static void completeRobotCycle() {
    if (Constants.kCurrentMode != Constants.Mode.SIM) {
      return;
    }

    record(Event.ROBOT_CYCLE);
    long completedCycles = counts[Event.ROBOT_CYCLE.ordinal()];
    if (completedCycles % 20 != 0) {
      return;
    }

    StringBuilder snapshot = new StringBuilder("SHOOTER_LIFECYCLE");
    for (Event event : Event.values()) {
      snapshot.append(' ').append(event).append('=').append(counts[event.ordinal()]);
    }
    snapshot.append(" DS_MODE=").append(driverStationMode());
    snapshot
        .append(" OP_RB=")
        .append(
            DriverStation.isJoystickConnected(Constants.kOperatorControllerPort)
                && DriverStation.getStickButton(Constants.kOperatorControllerPort, 6));
    snapshot
        .append(" TRACKING_SCHEDULED=")
        .append(
            trackingCommand != null && CommandScheduler.getInstance().isScheduled(trackingCommand));
    System.out.println(snapshot);
  }

  private static String driverStationMode() {
    if (!DriverStation.isDSAttached()) {
      return "DISCONNECTED";
    }
    if (DriverStation.isDisabled()) {
      return "DISABLED";
    }
    if (DriverStation.isTeleopEnabled()) {
      return "TELEOP";
    }
    if (DriverStation.isAutonomousEnabled()) {
      return "AUTO";
    }
    return "OTHER";
  }
}
