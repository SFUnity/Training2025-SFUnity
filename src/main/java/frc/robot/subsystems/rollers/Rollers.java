package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged;

    public Rollers(RollersIO rollersIO) {
        this.io = rollersIO;
    }

    @Override
    public void periodic() {
        io.UpdateInputs(inputs);
    }

    public Command RunVolts() {
        return run (()-> io.RunVolts(15));
    }

    public Command StopRollers() {
        return run (()-> io.RunVolts(0));

    public Command SlowRollers() {
        return run (()-> io.RunVolts(7));
    }
    }

}
