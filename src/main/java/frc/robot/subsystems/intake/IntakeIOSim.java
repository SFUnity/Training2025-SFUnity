package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim sim = new SingleJointedArmSim(
    DCMotor.getKrakenX60(1),
    50,
    0.5,
    armLengthMeters,
    minAngleRads,
    maxAngleRads,
    false,
    minAngleRads);
  
  private double appliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotAppliedVolts = appliedVolts;
  }

  @Override
  public void runPivot(double volts) {
    appliedVolts = volts;
  }
}
