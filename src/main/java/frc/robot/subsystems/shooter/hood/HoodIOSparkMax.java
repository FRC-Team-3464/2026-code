package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HoodIOSparkMax implements HoodIO {
  private final SparkMax motor;

  public HoodIOSparkMax(int motorID) {
    motor = new SparkMax(motorID, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // TODO Auto-generated method stub
    HoodIO.super.updateInputs(inputs);
  }
}
