package frc.robot.subsystems.ground;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GroundIOSim implements GroundIO {
        
    private final SingleJointedArmSim sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            0, 
            0, 
            0, 
            0, 
            0, 
            false, 
            0, 
            null);
}
 