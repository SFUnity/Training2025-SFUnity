package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class rollers extends SubsystemBase {

    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIO.RollersIOInputs();
    
    public rollers(RollersIO rollersIO) {
        this.io = rollersIO;
    }

    @Override
    public void periodic() {


        io.UpdateInputs(inputs);
    }
    public Command runRollersFast() {
       return run(() -> io.RunVolts(15));
    }
    public Command stopRollers() {
        return run(() -> io.RunVolts(0));
    }
        
    private Command runRollersSlow() {
        return run(() -> io.RunVolts(2));
    }
    }
    


