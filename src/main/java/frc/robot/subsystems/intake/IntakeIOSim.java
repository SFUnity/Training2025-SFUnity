package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          50,
          0.5,
          intakeLengthMeters,
          minAngleRads,
          maxAngleRads,
          false,
          minAngleRads);
    private final PIDController pid;
    private double pivotVolts = 0.0;
    private double rollersVolts = 0.0;

    public IntakeIOSim(){
        pid = new PIDController(kP.get(), 0.0, 0.0);
        sim.setState(minAngleRads, 0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        sim.update(0.02);
        inputs.pivotPositionRads = sim.getAngleRads() - minAngleRads;
        inputs.pivotVoltage = pivotVolts;
        inputs.pivotCurrent = sim.getCurrentDrawAmps();
        inputs.rollerVoltage = rollersVolts;
    }
    @Override
    public void runRollers(double volts){
        rollersVolts = volts;
    }
    @Override
    public void runPivot(double volts){
        sim.setInputVoltage(volts);
    }
    @Override
    public void setPivotPosition(double setPointRads){
        pivotVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads() - minAngleRads, setPointRads), -12, 12);
        runPivot(pivotVolts);
    }
}
