package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

private final DCMotor gearbox = DCMotor.getNEO(2);
private final DCMotorSim armSim;
private final DCMotorSim wheelSim;

//private final PIDController pid = new PIDController(1, 0, 0, Constants.kLoopPeriodSeconds);

private double armAppliedVolts = 0.0;
private double wheelAppliedVolts = 0.0;

public IntakeIOSim() {
    armSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, IntakeConstants.kPivotMotorGearRatio), 
            gearbox
        );

    wheelSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.025, IntakeConstants.kRollerMotorGearRatio), 
            gearbox
        );
}

@Override
public void updateInputs(IntakeIOInputs inputs) {

    armAppliedVolts = MathUtil.clamp(armAppliedVolts, -12.0, 12.0);
    wheelAppliedVolts = MathUtil.clamp(wheelAppliedVolts, -12.0, 12.0);

    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(0.02);

    wheelSim.setInputVoltage(wheelAppliedVolts);
    wheelSim.update(0.02);

    inputs.armPositionRad = armSim.getAngularPositionRotations();
    inputs.armVelocityRadPerSec = armSim.getAngularVelocityRPM();
    
    inputs.wheelPositionRad = wheelSim.getAngularPositionRotations();
    inputs.wheelVelocityRadPerSec = wheelSim.getAngularVelocityRPM();
}

@Override
public void setArmSpeed(double speed) {
    armAppliedVolts = 12 * speed;
}

@Override
public void setWheelSpeed(double speed) {
    wheelAppliedVolts = 12 * speed;
}

}
