package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class RollersIOTalonFX implements RollersIO {
  private final TalonFX rollerTalonFX = new TalonFX(RollersConstants.rollerID);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final NeutralOut stopOut = new NeutralOut();
  private final TalonFXConfiguration configs = new TalonFXConfiguration();

  public RollersIOTalonFX() {
    configs.CurrentLimits.SupplyCurrentLimit = 40;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerTalonFX.getConfigurator().apply(configs);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.rollerSpeed = rollerTalonFX.getVelocity().getValueAsDouble();
    inputs.rollerStatorCurrent = rollerTalonFX.getStatorCurrent().getValueAsDouble();
    inputs.rollerVolts = rollerTalonFX.getMotorVoltage().getValueAsDouble();
    inputs.rollerSupplyCurrent = rollerTalonFX.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    // modify with the double volts
    voltageOut.Output = volts;
    // apply to the motor
    rollerTalonFX.setControl(voltageOut);
  }

  @Override
  public void stop() {
    rollerTalonFX.setControl(stopOut);
  }
}
