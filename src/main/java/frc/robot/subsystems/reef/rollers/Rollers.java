package frc.robot.subsystems.reef.rollers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase{
    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs;

    public Rollers(RollersIO io, RollersIOInputsAutoLogged inputs){
        this.io = io;
        this.inputs = inputs;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    private void runIntakingRollers(){
        io.runVolts(RollersConstants.rollersIntakingSpeed);
    }
    private void runPlaceRollers(){
        io.runVolts(RollersConstants.rollersPlaceSpeed);
    }


    private void stopRollers(){
        io.stop();
    }

    public Command placeCoral(){
        return run(() -> {
            runPlaceRollers();
        }).withName("placeCoralRollers");
    }

    public Command intakeCoral(){
        return run(() -> {
            runIntakingRollers();
        }).withName("intakeCoralRollers");
    }
    //TODO: add "until when 2m distance sensor gets set up"
}
