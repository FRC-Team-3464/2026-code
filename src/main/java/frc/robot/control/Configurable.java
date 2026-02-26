package frc.robot.control;

/**
 * Represents any class that registers a group of bindings or settings during robot initialization.
 *
 * <p>Call {@link #configure()} once from RobotContainer during initialization.
 */
@FunctionalInterface
public interface Configurable {
  void configure();
}
