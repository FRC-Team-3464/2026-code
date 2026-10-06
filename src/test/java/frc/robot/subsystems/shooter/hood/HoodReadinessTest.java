package frc.robot.subsystems.shooter.hood;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Test;

class HoodReadinessTest {
  @Test
  void newGoalAndSensorLossInvalidateReadiness() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    try {
      RecordingHoodIO io = new RecordingHoodIO();
      Hood hood = new Hood(io);
      hood.setAngle(0.0);

      hood.periodic();
      assertFalse(hood.atGoal());
      SimHooks.stepTiming(0.22);
      hood.periodic();
      assertTrue(hood.atGoal());

      hood.setAngle(-0.4);
      assertFalse(hood.atGoal());
      io.positionRad = -0.4;
      hood.periodic();
      assertFalse(hood.atGoal());
      SimHooks.stepTiming(0.22);
      hood.periodic();
      assertTrue(hood.atGoal());

      io.connected = false;
      hood.periodic();
      assertFalse(hood.atGoal());
      io.connected = true;
      hood.setOpenLoop(0.0);
      assertFalse(hood.atGoal());
    } finally {
      SimHooks.resumeTiming();
    }
  }

  private static class RecordingHoodIO implements HoodIO {
    boolean connected = true;
    double positionRad;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
      inputs.connected = connected;
      inputs.positionRad = positionRad;
    }
  }
}
