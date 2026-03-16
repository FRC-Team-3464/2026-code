package frc.robot.control;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldConstants;
import frc.robot.util.Zone;

public class ZoneControls implements Configurable {

  private Zone leftTrenchZone = new Zone.RectangleZone(FieldConstants.LeftTrench.openingTopLeft.toTranslation2d(),
      FieldConstants.LeftTrench.openingTopRight.toTranslation2d()
          .plus(new Translation2d(FieldConstants.LeftTrench.depth, 0)));

  @Override
  public void configure() {
    Logger.recordOutput("TrenchZoneLeft",
        new Translation2d[] { FieldConstants.LeftTrench.openingTopLeft.toTranslation2d(),
            FieldConstants.LeftTrench.openingTopRight.toTranslation2d()
                .plus(new Translation2d(FieldConstants.LeftTrench.depth, 0)) });
  }
}
