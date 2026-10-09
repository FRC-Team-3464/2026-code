package frc.robot.control;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerLayout;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.TurretIO.*;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** HAL joystick packets exercise the actual adapters, bindings, defaults and command scheduler. */
class ControllerLayoutTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private JoystickSim driver;
  private JoystickSim operator;
  private RecordingDrive drive;
  private Shooter shooter;
  private Intake intake;
  private Indexer indexer;
  private final RecordingIntake intakeIO = new RecordingIntake();
  private final RecordingIndexer indexerIO = new RecordingIndexer();
  private final RecordingHood hoodIO = new RecordingHood();
  private final RecordingTurret turretIO = new RecordingTurret();
  private final RecordingFlywheel flywheelIO = new RecordingFlywheel();

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterAllSubsystems();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    driver = new JoystickSim(0);
    operator = new JoystickSim(1);
    connect(driver);
    connect(operator);
    drive = new RecordingDrive();
    shooter = new Shooter(turretIO, hoodIO, flywheelIO);
    intake = new Intake(intakeIO);
    indexer = new Indexer(indexerIO);
  }

  private static void connect(JoystickSim joystick) {
    joystick.setAxisCount(6);
    joystick.setButtonCount(10);
    joystick.setPOVCount(1);
    joystick.setPOV(0, -1);
  }

  private void configure(ControllerLayout layout) {
    DriverController d = new DriverController.XboxDriverController(0);
    DriverController o = new DriverController.XboxDriverController(1);
    if (layout == ControllerLayout.SINGLE_CONTROLLER) {
      SingleDriverControls controls = new SingleDriverControls(d, drive, shooter, intake, indexer);
      controls.configure();
      new DefaultControls(d, drive, shooter, controls::rotationInput, controls::holdHood)
          .configure();
    } else {
      DualDriverControls controls = new DualDriverControls(d, o, drive, shooter, intake, indexer);
      controls.configure();
      new DefaultControls(d, drive, shooter, controls::rotationInput, controls::holdHood)
          .configure();
    }
    tick();
    DriverStationSim.setEnabled(true);
    tick();
  }

  private void tick() {
    for (int i = 0; i < 3; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(0.02);
      scheduler.run();
      shooter.getTurret().periodicAfterScheduler();
    }
  }

  @AfterEach
  void cleanup() {
    DriverStationSim.setEnabled(false);
    tick();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterAllSubsystems();
    SimHooks.resumeTiming();
  }

  @Test
  void rightStickRequiresNeutralAtBothModifierTransitionsAndDpadStillDrives() {
    configure(ControllerLayout.SINGLE_CONTROLLER);
    driver.setRawAxis(4, .8);
    tick();
    assertTrue(Math.abs(drive.speeds.omegaRadiansPerSecond) > 0);
    driver.setRawAxis(2, 1); // LT while steering: stop chassis, do not turn turret yet.
    tick();
    assertEquals(0, drive.speeds.omegaRadiansPerSecond);
    assertEquals(0, turretIO.output);
    driver.setRawAxis(4, 0);
    tick();
    driver.setRawAxis(4, .8);
    driver.setRawAxis(5, -.7);
    tick();
    assertTrue(turretIO.output > 0 && turretIO.output <= .05);
    assertTrue(hoodIO.output > 0);
    assertEquals(0, drive.speeds.omegaRadiansPerSecond);
    driver.setPOV(0, 0);
    tick();
    assertTrue(Math.abs(drive.speeds.vxMetersPerSecond) > 0);
    driver.setPOV(0, -1);
    driver.setRawAxis(2, 0);
    tick();
    assertEquals(0, turretIO.output);
    assertEquals(0, drive.speeds.omegaRadiansPerSecond);
    driver.setRawAxis(4, 0);
    driver.setRawAxis(5, 0);
    tick();
    driver.setRawAxis(4, -.6);
    tick();
    assertTrue(Math.abs(drive.speeds.omegaRadiansPerSecond) > 0);
  }

  @Test
  void manualAimHoldsHoodAndRequiresNewRbPressWhileFlywheelAndFeedRemainIndependent() {
    configure(ControllerLayout.SINGLE_CONTROLLER);
    driver.setRawButton(6, true);
    driver.setRawAxis(3, 1);
    tick();
    assertEquals(TurretIOOutputMode.CLOSED_LOOP, turretIO.mode);
    assertTrue(flywheelIO.velocityMode);
    assertNotEquals(0, indexerIO.output);
    driver.setRawAxis(2, 1);
    tick();
    driver.setRawAxis(5, -.7);
    tick();
    assertTrue(hoodIO.output > 0);
    assertTrue(flywheelIO.velocityMode);
    driver.setRawAxis(5, 0);
    tick();
    assertEquals(-1, hoodIO.target);
    driver.setRawAxis(2, 0);
    tick();
    assertEquals(TurretIOOutputMode.OPEN_LOOP, turretIO.mode);
    assertEquals(-1, hoodIO.target);
    driver.setRawButton(6, false);
    tick();
    assertFalse(flywheelIO.velocityMode);
    assertEquals(0, hoodIO.target);
    driver.setRawButton(6, true);
    tick();
    assertEquals(TurretIOOutputMode.CLOSED_LOOP, turretIO.mode);
  }

  @Test
  void intakePriorityResumesAndPortOneHasNoBindingsInSingleMode() {
    configure(ControllerLayout.SINGLE_CONTROLLER);
    operator.setRawButton(5, true);
    operator.setRawButton(6, true);
    tick();
    assertEquals(0, intakeIO.roller);
    assertFalse(flywheelIO.velocityMode);
    driver.setRawButton(5, true);
    tick();
    double collection = intakeIO.roller;
    assertNotEquals(0, collection);
    driver.setRawButton(4, true);
    tick();
    assertEquals(0, intakeIO.roller);
    assertNotEquals(0, intakeIO.pivot);
    driver.setRawButton(8, true);
    tick();
    assertEquals(0, intakeIO.pivot);
    driver.setRawButton(4, false);
    tick();
    assertNotEquals(0, intakeIO.pivot);
    driver.setRawButton(8, false);
    tick();
    assertEquals(collection, intakeIO.roller);
    driver.setRawButton(1, true);
    tick();
    assertEquals(-collection, intakeIO.roller);
    driver.setRawButton(1, false);
    tick();
    assertEquals(collection, intakeIO.roller);
  }

  @Test
  void disableStopsManualCommandsAndAutonomousKeepsOwnership() {
    configure(ControllerLayout.SINGLE_CONTROLLER);
    driver.setRawAxis(2, 1);
    tick();
    driver.setRawAxis(4, .8);
    driver.setRawAxis(3, 1);
    tick();
    assertTrue(turretIO.output > 0);
    DriverStationSim.setEnabled(false);
    tick();
    assertEquals(0, turretIO.output);
    assertEquals(0, indexerIO.output);
    DriverStationSim.setEnabled(true);
    tick();
    assertEquals(0, turretIO.output); // Shared stick must center before resuming manual motion.
    driver.setRawAxis(4, 0);
    driver.setRawAxis(2, 0);
    tick();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var auto = Commands.startEnd(() -> shooter.getHood().setAngle(-2), () -> {}, shooter.getHood());
    scheduler.schedule(auto);
    tick();
    assertTrue(auto.isScheduled());
    assertEquals(-2, hoodIO.target);
    assertTrue(hoodIO.closedLoop);
    assertEquals(0, drive.speeds.omegaRadiansPerSecond);
    auto.cancel();
  }

  @Test
  void headingResetNeedsFreshPressAndTestModeDoesNotRunBindings() {
    driver.setRawButton(3, true);
    configure(ControllerLayout.SINGLE_CONTROLLER);
    assertEquals(0, drive.headingResets);
    driver.setRawButton(3, false);
    tick();
    driver.setRawButton(3, true);
    tick();
    assertEquals(1, drive.headingResets);
    driver.setRawButton(3, false);
    driver.setRawButton(2, true);
    tick();
    assertEquals(1, drive.xLocks);
    driver.setRawButton(2, false);
    driver.setRawButton(5, true);
    tick();
    assertNotEquals(0, intakeIO.roller);
    DriverStationSim.setTest(true);
    driver.setRawButton(5, true);
    driver.setPOV(0, 0);
    tick();
    assertEquals(0, intakeIO.roller);
    assertEquals(0, drive.speeds.vxMetersPerSecond);
  }

  @Test
  void dualLayoutKeepsOperatorBindingsAndDriverSwerve() {
    configure(ControllerLayout.TWO_CONTROLLERS);
    driver.setRawButton(5, true);
    driver.setRawButton(6, true);
    tick();
    assertEquals(0, intakeIO.roller);
    assertFalse(flywheelIO.velocityMode);
    operator.setRawButton(5, true);
    tick();
    assertNotEquals(0, intakeIO.roller);
    operator.setRawButton(3, true); // Existing X deploy.
    tick();
    assertNotEquals(0, intakeIO.pivot);
    assertEquals(0, intakeIO.roller);
    operator.setRawButton(3, false);
    operator.setRawButton(6, true);
    driver.setPOV(0, 90);
    tick();
    assertNotEquals(0, intakeIO.roller);
    assertTrue(flywheelIO.velocityMode);
    assertTrue(Math.abs(drive.speeds.vyMetersPerSecond) > 0);
  }

  private static class RecordingDrive extends Drive {
    ChassisSpeeds speeds = new ChassisSpeeds();
    int headingResets, xLocks;

    @Override
    public edu.wpi.first.wpilibj2.command.Command resetHeading() {
      return super.resetHeading().beforeStarting(() -> headingResets++);
    }

    @Override
    public void stopWithX() {
      xLocks++;
      super.stopWithX();
    }

    RecordingDrive() {
      super(
          new GyroIO() {},
          new ModuleIOSim(DriveConstants.TunerConstants.FrontLeft),
          new ModuleIOSim(DriveConstants.TunerConstants.FrontRight),
          new ModuleIOSim(DriveConstants.TunerConstants.BackLeft),
          new ModuleIOSim(DriveConstants.TunerConstants.BackRight));
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
      this.speeds = speeds;
      super.runVelocity(speeds);
    }
  }

  private static class RecordingIntake implements IntakeIO {
    double roller, pivot;

    @Override
    public void setWheelSpeed(double value) {
      roller = value;
    }

    @Override
    public void setPivotSpeed(double value) {
      pivot = value;
    }
  }

  private static class RecordingIndexer extends IndexerIOSim {
    double output;

    @Override
    public void setThroatOpenLoop(double value) {
      output = value;
      super.setThroatOpenLoop(value);
    }

    @Override
    public void stop() {
      output = 0;
      super.stop();
    }
  }

  private static class RecordingHood implements HoodIO {
    double output, target;
    boolean closedLoop;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      inputs.connected = true;
      inputs.positionRad = -1;
    }

    @Override
    public void setOpenLoop(double value) {
      output = value;
      closedLoop = false;
    }

    @Override
    public void setAngle(double value) {
      target = value;
      closedLoop = true;
    }
  }

  private static class RecordingTurret extends TurretIOSim {
    double output;
    TurretIOOutputMode mode;

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
      mode = outputs.mode;
      output = outputs.openLoopOutput;
      super.applyOutputs(outputs);
    }
  }

  private static class RecordingFlywheel extends FlywheelIOSim {
    boolean velocityMode;

    @Override
    public void setVelocity(double value) {
      velocityMode = true;
      super.setVelocity(value);
    }

    @Override
    public void setOpenLoop(double value) {
      velocityMode = false;
      super.setOpenLoop(value);
    }
  }
}
