package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class rollers extends SubsystemBase {

    private final rollersIO io;
    private final RollersIOInputs inputs = new RollersIOInputs();

    public rollers(rollersIO rollers) {
        this.io = rollers;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command runRollersFast(){
        return run(() -> io.runVolts(15));
    }

      public Command stopRollers(){
        return run(() -> io.runVolts(0));
    }
   
}
