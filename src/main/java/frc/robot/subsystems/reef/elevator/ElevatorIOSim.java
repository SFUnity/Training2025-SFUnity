package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim {
  private final PIDController pid;
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

  public ElevatorIOSim() {
    pid = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }
}
