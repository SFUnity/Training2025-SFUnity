package frc.robot.subsystems.Rollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

    public Rollers (RollersIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command intake() {
        return run(() -> io.runVolts(0.6));
    }
    
    public Command stop() {
        return run(() -> io.runVolts(0.0));
    }

    
}
