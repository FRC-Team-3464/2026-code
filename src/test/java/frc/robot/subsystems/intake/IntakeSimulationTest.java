package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotVisualizer;
import frc.robot.visualization.IntakeModelGeometry;
import java.nio.file.Files;
import java.nio.file.Path;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSimulationTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private IntakeIOSim io;
  private Intake intake;

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterAllSubsystems();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.notifyNewData();
    io = new IntakeIOSim();
    intake = new Intake(io);
  }

  @AfterEach
  void cleanup() {
    scheduler.cancelAll();
    DriverStationSim.setEnabled(false);
    tick(1);
    scheduler.unregisterAllSubsystems();
    RobotVisualizer.getInstance().setIntakeExtensionFraction(0);
    SimHooks.resumeTiming();
  }

  private void tick(int cycles) {
    for (int i = 0; i < cycles; i++) {
      DriverStationSim.notifyNewData();
      SimHooks.stepTiming(0.02);
      scheduler.run();
    }
  }

  private double displayedFraction() {
    var poses = RobotVisualizer.getInstance().getComponentPoses();
    assertEquals(3, poses.length);
    assertEquals(IntakeModelGeometry.RETRACTED_POSITION.getZ(), poses[2].getZ());
    return (IntakeModelGeometry.RETRACTED_POSITION.getX() - poses[2].getX())
        / IntakeModelGeometry.TRAVEL_METERS;
  }

  @Test
  void actualAutoIntakeCycleDrivesTheThirdComponent() throws Exception {
    NamedCommands.registerCommand("DeployIntake", intake.deployOpenLoop().withTimeout(2));
    NamedCommands.registerCommand("RetractIntake", intake.retractOpenLoop().withTimeout(2));
    io.setPivotSpeed(IntakeConstants.kPivotMotorSpeed);
    tick(110);
    io.setPivotSpeed(0);
    assertEquals(1, displayedFraction(), 1e-8);

    JSONObject auto =
        (JSONObject)
            new JSONParser()
                .parse(
                    Files.readString(
                        Path.of(
                            "src/main/deploy/pathplanner/autos/URI-RT-OP-SUTO-CYCLE-INTAKE.auto")));
    JSONArray steps = children((JSONObject) auto.get("command"));
    JSONArray shooting = children((JSONObject) steps.get(steps.size() - 1));
    var cycle =
        CommandUtil.commandFromJson((JSONObject) shooting.get(shooting.size() - 1), false, false);
    scheduler.schedule(cycle);
    tick(125); // 2.5 seconds: still waiting for the first retract.
    assertEquals(1, displayedFraction(), 1e-8);
    tick(130); // 5.1 seconds: two-second retract has completed.
    assertEquals(0, displayedFraction(), 0.06);
    tick(50); // 6.1 seconds: one-second extension has reached about half travel.
    assertEquals(0.5, displayedFraction(), 0.08);
    tick(50);
    assertEquals(0, displayedFraction(), 0.08);
    tick(50);
    assertEquals(0.5, displayedFraction(), 0.08);
    tick(80);
    assertFalse(cycle.isScheduled());
    assertEquals(0, displayedFraction(), 0.03);
    tick(100);
    assertEquals(0, displayedFraction(), 0.03);
  }

  private static JSONArray children(JSONObject command) {
    return (JSONArray) ((JSONObject) command.get("data")).get("commands");
  }

  @Test
  void disableHoldsTravelAndClearsRequestsWithoutChangingRealFeedback() {
    io.setPivotSpeed(IntakeConstants.kPivotMotorSpeed);
    io.setWheelSpeed(0.8);
    tick(50);
    double halfway = displayedFraction();
    assertEquals(0.5, halfway, 0.02);
    DriverStationSim.setEnabled(false);
    tick(100);
    assertEquals(halfway, displayedFraction());
    DriverStationSim.setEnabled(true);
    tick(50);
    assertEquals(halfway, displayedFraction());
    // Rollers alone do not move the carriage.
    io.setWheelSpeed(0.8);
    tick(50);
    assertEquals(halfway, displayedFraction());
    io.setPivotSpeed(-IntakeConstants.kPivotMotorSpeed);
    tick(200);
    assertEquals(0, displayedFraction(), 1e-8);
    var inputs = new IntakeIO.IntakeIOInputs();
    io.updateInputs(inputs);
    assertTrue(inputs.simExtensionAvailable);
    assertFalse(inputs.leftPivotConnected);
    assertEquals(0, inputs.leftPivotVelocityRadPerSec);
  }
}
