package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class IndexerTest {
  @Test
  void guardedFeedPausesResumesAndStopsWhenInterrupted() {
    RecordingIndexerIO io = new RecordingIndexerIO();
    AtomicBoolean shooterReady = new AtomicBoolean(false);
    Command feed = new Indexer(io).indexWhileReady(shooterReady::get);

    feed.initialize();
    feed.execute();
    assertEquals(0.0, io.throatOutput);
    assertEquals(0.0, io.tongueOutput);

    shooterReady.set(true);
    feed.execute();
    assertEquals(-IndexerConstants.kGutsMotorSpeed, io.throatOutput);
    assertEquals(IndexerConstants.kGutsMotorSpeed, io.tongueOutput);

    shooterReady.set(false);
    feed.execute();
    assertEquals(0.0, io.throatOutput);
    assertEquals(0.0, io.tongueOutput);

    shooterReady.set(true);
    feed.execute();
    assertEquals(IndexerConstants.kGutsMotorSpeed, io.tongueOutput);

    feed.end(true);
    assertEquals(0.0, io.throatOutput);
    assertEquals(0.0, io.tongueOutput);
  }

  private static class RecordingIndexerIO implements IndexerIO {
    double throatOutput;
    double tongueOutput;

    @Override
    public void setThroatOpenLoop(double output) {
      throatOutput = output;
    }

    @Override
    public void setTongueOpenLoop(double output) {
      tongueOutput = output;
    }

    @Override
    public void stop() {
      throatOutput = 0.0;
      tongueOutput = 0.0;
    }
  }
}
