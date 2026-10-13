package frc.robot.subsystems.shooter.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import org.junit.jupiter.api.Test;

class TurretLimitTest {
  @Test
  void limitsBlockOutwardMotionAndAllowInwardRecovery() {
    HAL.initialize(500, 0);
    RecordingIO io = new RecordingIO();
    Turret turret = new Turret(io);
    try {
      for (double position :
          new double[] {
            TurretConstants.kMaxTurretAngleRad, TurretConstants.kMaxTurretAngleRad + .01
          }) {
        io.position = position;
        turret.periodic();
        checkOutput(turret, io, .05, 0);
        checkOutput(turret, io, -.05, -.05);
        checkOutput(turret, io, 0, 0);
      }
      for (double position :
          new double[] {
            TurretConstants.kMinTurretAngleRad, TurretConstants.kMinTurretAngleRad - .01
          }) {
        io.position = position;
        turret.periodic();
        checkOutput(turret, io, -.05, 0);
        checkOutput(turret, io, .05, .05);
        checkOutput(turret, io, 0, 0);
      }
      io.position = 0;
      turret.periodic();
      checkOutput(turret, io, .05, .05);
      checkOutput(turret, io, -.05, -.05);
      checkOutput(turret, io, 2, 1);
      checkOutput(turret, io, -2, -1);
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(turret);
    }
  }

  private void checkOutput(Turret turret, RecordingIO io, double requested, double expected) {
    turret.setOpenLoop(requested);
    turret.periodicAfterScheduler();
    assertEquals(expected, io.output, 1e-9);
  }

  private static class RecordingIO implements TurretIO {
    double position;
    double output;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
      inputs.connected = true;
      inputs.positionRad = position;
    }

    @Override
    public void applyOutputs(TurretIOOutputs outputs) {
      output = outputs.openLoopOutput;
    }
  }
}
