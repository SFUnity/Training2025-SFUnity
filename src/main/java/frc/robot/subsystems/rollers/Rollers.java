package frc.robot.subsystems.rollers;

public class Rollers 

public class Rollers extends SubsystemBase {
    private final RollersIO io;
    private final RollersIO.RollersIOIinputs inputs = new RollersIO.RollersIOIinputs();

    public Rollers(RollersIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command command() {
        return run(() -> io.runVolts());
    }
    }

