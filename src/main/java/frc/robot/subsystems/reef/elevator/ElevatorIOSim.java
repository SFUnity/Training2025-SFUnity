package frc.robot.subsystems.reef.elevator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
public class ElevatorIOSim {
    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getNEO(1), 
        1.0, //gearbox
        ElevatorConstants.carrageMass, 
        ElevatorConstants.minHeight, 
        ElevatorConstants.maxHeight,
        true, 
        0 
    );

    public ElevatorIOSim(){

    }


    
}
