package frc.robot.subsystems.ground;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ground extends SubsystemBase {
    private final GroundIO io;
    public Ground(GroundIO io){
        this.io = io;
    }
}
