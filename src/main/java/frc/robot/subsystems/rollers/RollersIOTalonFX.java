package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX rollerTalonFX = new TalonFX(RollersConstants.rollerID);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VoltageOut stopOut = new VoltageOut(0.0);

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.rollerSpeed = rollerTalonFX.getVelocity().getValueAsDouble();
    inputs.rollerStatorCurrent = rollerTalonFX.getStatorCurrent().getValueAsDouble();
    inputs.rollerSupplyCurrent = rollerTalonFX.getSupplyCurrent().getValueAsDouble();
    inputs.rollerVolts = rollerTalonFX.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    voltageOut.Output = volts;
    rollerTalonFX.setControl(voltageOut);
  }

  @Override
  public void stop() {
    rollerTalonFX.setControl(stopOut);
  }
}
