package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {

    private SparkMax pivotMotor = new SparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
    private SparkMax rollerMotor = new SparkMax(IntakeConstants.kRollerMotorID, MotorType.kBrushless);

    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    public IntakeIOSparkMax() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void runPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void runRollerMotor(double speed) {
        rollerMotor.set(speed);
    }
}
