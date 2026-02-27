// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.OdometryObservation;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.SwerveMod;
import frc.robot.subsystems.drive.SwerveMod.ModuleName;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDMode;

public class RobotContainer {
  private final XboxController driver = new XboxController(0);

  private final Drive drive;
  private final LEDs ledsFlashWhite;
  private final LEDs ledsFlashYellow;
  private final LEDs ledsSolidRed;
  private final LEDs ledsSolidGreen;
  private final LEDs ledsRedAndPurpleGradient;
  private final LEDs ledsRainbowChroma;
  private final LEDs ledsOff;

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(null, null);
    } else {
      drive =
          new Drive(
              new SwerveMod[] {
                new SwerveMod(new ModuleIOSim(), ModuleName.FRONT_LEFT),
                new SwerveMod(new ModuleIOSim(), ModuleName.FRONT_RIGHT),
                new SwerveMod(new ModuleIOSim(), ModuleName.BACK_LEFT),
                new SwerveMod(new ModuleIOSim(), ModuleName.BACK_RIGHT),
              },
              new GyroIO() {});
    }

    ledsFlashWhite = new LEDs(LEDMode.FLASH_WHITE);
    ledsFlashYellow = new LEDs(LEDMode.FLASH_YELLOW);
    ledsSolidRed = new LEDs(LEDMode.SOLID_RED);
    ledsSolidGreen = new LEDs(LEDMode.SOLID_GREEN);
    ledsRedAndPurpleGradient = new LEDs(LEDMode.RED_AND_PURPLE_GRADIENT);
    ledsRainbowChroma = new LEDs(LEDMode.RAINBOW_CHROMA);
    ledsOff = new LEDs(LEDMode.OFF);

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false));

    Constants.OperatorConstants.testButton.whileTrue(ledsSolidGreen.setLEDPattern());

  }

  public void robotPeriodic() {
    OdometryObservation obs =
        new OdometryObservation(
            Timer.getTimestamp(), drive.getModulePositions(), drive.getRawGyroRotation());
    RobotState.getInstance().addOdometryObservation(obs);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
