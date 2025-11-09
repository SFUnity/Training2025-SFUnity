package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.errorprone.annotations.Var;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class IntakeIOReal implements IntakeIO{
    private final TalonFX pivot = new TalonFX(1);
    private final TalonFX roller = new TalonFX(2);
    @Override
    public void updateInput(IntakeIOInputs inputs){
        inputs.pivotCurrentPositionDeg = pivot.getPosition().getValueAsDouble();
        inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
        inputs.rollerCurrent = roller.getSupplyCurrent().getValueAsDouble();
        inputs.pivotCurrentAmps = pivot.getSupplyCurrent().getValueAsDouble();
        inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
        inputs.rollerPositionRad = roller.getPosition().getValueAsDouble();
    }
}
