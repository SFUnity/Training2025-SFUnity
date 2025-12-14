package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX pivot = new TalonFX(1);
  private final TalonFX roller = new TalonFX(2);
  private final PIDController controller;

  public IntakeIOReal() {
    controller = new PIDController(kP.get(), 0, kD.get());
  }

  @Override
  public void updateInput(IntakeIOInputs inputs) {
    inputs.pivotCurrentPositionDeg = pivot.getPosition().getValueAsDouble();
    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrent = roller.getSupplyCurrent().getValueAsDouble();
    inputs.pivotCurrentAmps = pivot.getSupplyCurrent().getValueAsDouble();
    inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    inputs.rollerPositionRad = roller.getPosition().getValueAsDouble();
  }

  @Override
  public void runRollers(double volts) {
    roller.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(double angle) {
    double volts = controller.calculate(pivot.getPosition().getValueAsDouble(), angle);
    volts = MathUtil.clamp(volts, -12.0, high);
    pivot.setVoltage(volts);
  }
}
