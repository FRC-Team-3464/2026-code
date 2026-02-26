package frc.robot.control;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverControls implements Configurable {
    @AutoLogOutput(key = "Control/DriverControls/mode")
    private DriverMode mode = DriverMode.ONE_DRIVER;

    public enum DriverMode {
        ONE_DRIVER,
        TWO_DRIVERS;
    }

    private final DriverController driver;
    private final Joystick operator;
    private final Drive drive;
    private final Shooter leftShooter;
    private final Shooter rightShooter;

    public DriverControls(
            DriverController driver,
            Joystick operator,
            Drive drive,
            Shooter leftShooter,
            Shooter rightShooter) {
        this.driver = driver;
        this.operator = operator;
        this.drive = drive;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
    }

    @Override
    public void configure() {

        // Neutral controls (regardless of whether we are in one or two driver mode)
        driver.xSquare()
                .onTrue(Shooter.shootBothAtHub(leftShooter, rightShooter));

        configureOneDriver();
        configureTwoDrivers();
    }

    /*
     * Driver Bindings:
     *
     * <p>LB: Toggle deploy/retract intake LT: Spin intake RB: Shoot LT + A:
     * backspin intake RT:
     * Climb RT + A: Unclimb X: reset Gyro D-Pad: CrabWalk LB + RB + Y: Aux Handoff
     *
     */

    private void configureOneDriver() {
        driver.rightBumper().and(this::isOneDriver)
                .onTrue(Shooter.shootBothAtHub(leftShooter, rightShooter));
    }

    /*
     * <p>Back up Operator Controls:
     *
     * <p>Pancake up + down: Pitch of turrets Pancake left + right: rotation of
     * turrets trigger
     * button: Fires fuel from turrets
     *
     * <p>button 7: deploy intake button 8: run intake button 9: retract intake
     *
     * <p>button 6: climber up button 4: climber down
     *
     * <p>thumb button: Driver Handoff
     */
    private void configureTwoDrivers() {
        
    }

    private boolean isOneDriver() {
        return mode == DriverMode.ONE_DRIVER;
    }

    private boolean isTwoDrivers() {
        return mode == DriverMode.TWO_DRIVERS;
    }

    public void setMode(DriverMode mode) {
        this.mode = mode;
        configure();
    }

    public DriverMode getMode() {
        return mode;
    }
}
