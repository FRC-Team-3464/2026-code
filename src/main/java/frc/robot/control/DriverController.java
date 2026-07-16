package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Abstracts controller input so DriverControls works with any supported controller type without
 * caring about the underlying hardware.
 */
public interface DriverController {

  Trigger aCross();

  Trigger bCircle();

  Trigger xSquare();

  Trigger yTriangle();

  Trigger leftBumper();

  Trigger rightBumper();

  Trigger leftTrigger();

  Trigger rightTrigger();

  Trigger dPadUp();

  Trigger dPadUpLeft();

  Trigger dPadUpRight();

  Trigger dPadDown();

  Trigger dPadDownLeft();

  Trigger dPadDownRight();

  Trigger dPadLeft();

  Trigger dPadRight();

  double getLeftX();

  double getLeftY();

  double getRightX();

  double getRightY();

  void rumble(RumbleType rumbleType, double intensity);

  class XboxDriverController implements DriverController {
    private final CommandXboxController controller;

    public XboxDriverController(int controllerID) {
      this.controller = new CommandXboxController(controllerID);
    }

    @Override
    public Trigger aCross() {
      return controller.a();
    }

    @Override
    public Trigger bCircle() {
      return controller.b();
    }

    @Override
    public Trigger xSquare() {
      return controller.x();
    }

    @Override
    public Trigger yTriangle() {
      return controller.y();
    }

    @Override
    public Trigger leftBumper() {
      return controller.leftBumper();
    }

    @Override
    public Trigger rightBumper() {
      return controller.rightBumper();
    }

    @Override
    public Trigger leftTrigger() {
      return controller.leftTrigger();
    }

    @Override
    public Trigger rightTrigger() {
      return controller.rightTrigger();
    }

    @Override
    public Trigger dPadUp() {
      return controller.povUp();
    }

    @Override
    public Trigger dPadUpLeft() {
      return controller.povUpLeft();
    }

    @Override
    public Trigger dPadUpRight() {
      return controller.povUpRight();
    }

    @Override
    public Trigger dPadDown() {
      return controller.povDown();
    }

    @Override
    public Trigger dPadDownLeft() {
      return controller.povDownLeft();
    }

    @Override
    public Trigger dPadDownRight() {
      return controller.povDownRight();
    }

    @Override
    public Trigger dPadLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger dPadRight() {
      return controller.povRight();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }

    @Override
    public void rumble(RumbleType rumbleType, double intensity) {
      controller.setRumble(rumbleType, intensity);
    }
  }

  class PS5DriverController implements DriverController {
    private final CommandPS5Controller controller;

    public PS5DriverController(int controllerID) {
      this.controller = new CommandPS5Controller(controllerID);
    }

    @Override
    public Trigger aCross() {
      return controller.cross();
    }

    @Override
    public Trigger bCircle() {
      return controller.circle();
    }

    @Override
    public Trigger xSquare() {
      return controller.square();
    }

    @Override
    public Trigger yTriangle() {
      return controller.triangle();
    }

    @Override
    public Trigger leftBumper() {
      return controller.L1();
    }

    @Override
    public Trigger rightBumper() {
      return controller.R1();
    }

    @Override
    public Trigger leftTrigger() {
      return controller.L2();
    }

    @Override
    public Trigger rightTrigger() {
      return controller.R2();
    }

    @Override
    public Trigger dPadUp() {
      return controller.povUp();
    }

    @Override
    public Trigger dPadUpLeft() {
      return controller.povUpLeft();
    }

    @Override
    public Trigger dPadUpRight() {
      return controller.povUpRight();
    }

    @Override
    public Trigger dPadDown() {
      return controller.povDown();
    }

    @Override
    public Trigger dPadDownLeft() {
      return controller.povDownLeft();
    }

    @Override
    public Trigger dPadDownRight() {
      return controller.povDownRight();
    }

    @Override
    public Trigger dPadLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger dPadRight() {
      return controller.povRight();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }

    @Override
    public void rumble(RumbleType rumbleType, double intensity) {
      controller.setRumble(rumbleType, intensity);
    }
  }

  class PS4DriverController implements DriverController {
    private final CommandPS4Controller controller;

    public PS4DriverController(int controllerID) {
      this.controller = new CommandPS4Controller(controllerID);
    }

    @Override
    public Trigger aCross() {
      return controller.cross();
    }

    @Override
    public Trigger bCircle() {
      return controller.circle();
    }

    @Override
    public Trigger xSquare() {
      return controller.square();
    }

    @Override
    public Trigger yTriangle() {
      return controller.triangle();
    }

    @Override
    public Trigger leftBumper() {
      return controller.L1();
    }

    @Override
    public Trigger rightBumper() {
      return controller.R1();
    }

    @Override
    public Trigger leftTrigger() {
      return controller.L2();
    }

    @Override
    public Trigger rightTrigger() {
      return controller.R2();
    }

    @Override
    public Trigger dPadUp() {
      return controller.povUp();
    }

    @Override
    public Trigger dPadUpLeft() {
      return controller.povUpLeft();
    }

    @Override
    public Trigger dPadUpRight() {
      return controller.povUpRight();
    }

    @Override
    public Trigger dPadDown() {
      return controller.povDown();
    }

    @Override
    public Trigger dPadDownLeft() {
      return controller.povDownLeft();
    }

    @Override
    public Trigger dPadDownRight() {
      return controller.povDownRight();
    }

    @Override
    public Trigger dPadLeft() {
      return controller.povLeft();
    }

    @Override
    public Trigger dPadRight() {
      return controller.povRight();
    }

    @Override
    public double getLeftX() {
      return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
      return controller.getLeftY();
    }

    @Override
    public double getRightX() {
      return controller.getRightX();
    }

    @Override
    public double getRightY() {
      return controller.getRightY();
    }

    @Override
    public void rumble(RumbleType rumbleType, double intensity) {
      controller.setRumble(rumbleType, intensity);
    }
  }
}
