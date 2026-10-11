package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Flywheels {

public class Flywheels extends SubsystemBase {
    private final FlywheelsIO io;
    private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    public Flywheels(FlywheelsIO flywheelsIO) {
        this.io = flywheelsIO;
    }

    @Override
    public void periodic() {
        io.UpdateInputs(inputs);
    }

    //FlywheelsTop
    private Command RunFlywheelsFast() {
      return run (()-> io.RunVoltsTop(+15));
    }

    private Command stopFlywheels() {
        return run (()-> io.RunVoltsTop(0));
    } 

    private Command RunFlywheelsSlow(){
        return run (()-> io.RunVoltsTop(+5));
    }

    //FlywheelsBottom
    private Command RunFlywheelsFast() {
      return run (()-> io.RunVoltsBottom(-15));
    }

    private Command stopFlywheels() {
        return run (()-> io.RunVoltsBottom(0));
    } 

    private Command RunFlywheelsSlow(){
        return run (()-> io.RunVoltsBottom(-5));
    }

}
}

/*idk if it works of if thats how it spins but
     i think the top flywheel will spin one way because if the "+" sign and the bottom will spin the opposite way because of the "-" sign
     i want it to work please please please
     */