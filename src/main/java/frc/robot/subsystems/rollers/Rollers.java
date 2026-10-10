package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Rollers extends SubsystemBase {
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutologged();
    
    public Rollers(RollersIO rollersIO) {
        this.io = rollersIO;
    }

    @Override 
    public void periodic() {
        io.updateInputs(inputs);
    }
    private Command runRollersFast() {
        return run (()-> io.runVolts(15));
    }
    private Command stopRollers() {
        return run(()-> io.runVolts(0));
    }
    private Command runRollersSlow() {
        return run(()-> io.runVolts(5));
    }
}
