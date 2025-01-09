package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Meters
;
public class ElevatorIOSim extends SubsystemBase{
  private final PIDController pid;
  private final ElevatorIOInputsAutoLogged inputs;

  private double appliedVolts = 0.0;
  private final ElevatorSim sim = 
      new ElevatorSim(
          DCMotor.getNEO(1),
          1,
          ElevatorConstants.drumRadius,
          ElevatorConstants.carrageMass,
          ElevatorConstants.minHeight,
          ElevatorConstants.maxHeight,
          true,
          0);

  public ElevatorIOSim(ElevatorIOInputsAutoLogged inputs) {
    this.inputs = inputs;
    pid = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void periodic(){
    inputs.position = Meters.of(sim.getPositionMeters());
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }
}
