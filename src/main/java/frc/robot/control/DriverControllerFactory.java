package frc.robot.control;

import frc.robot.Constants.Mode;

/** Creates the controller adapter selected for a driver or operator port. */
public final class DriverControllerFactory {
  /** Controller button and axis layout expected on a Driver Station port. */
  public enum Profile {
    XBOX,
    PS4,
    PS5
  }

  private DriverControllerFactory() {}

  /**
   * Creates one controller adapter for the configured port and layout.
   *
   * <p>The real layout is selected at startup, not guessed from a temporarily disconnected
   * controller. SIM always uses the Xbox layout so a keyboard can be mapped to this port as a
   * virtual Xbox-style joystick in the WPILib simulation GUI.
   *
   * @param mode current robot mode
   * @param realProfile button and axis layout to use on the physical robot
   * @param port Driver Station joystick port
   * @return controller adapter for the selected layout
   */
  public static DriverController create(Mode mode, Profile realProfile, int port) {
    Profile profile =
        switch (mode) {
          case REAL -> realProfile;
          case SIM, REPLAY -> Profile.XBOX;
        };
    return switch (profile) {
      case XBOX -> new DriverController.XboxDriverController(port);
      case PS4 -> new DriverController.PS4DriverController(port);
      case PS5 -> new DriverController.PS5DriverController(port);
    };
  }
}
