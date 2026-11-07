package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private final RollersIO io;
    private RollersIOInputsaAutoLogged inputs = new RollersIOInputs();
    private RollersState state = RollersState.STOP;

    public Rollers(RollersIO io) {
        this.io = io;
    }
    public enum RollersState {
        STOP, INTAKE, OUTTAKE

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    pulbic void setState(RollersState state) {
        this.state = state;
        switch (state) [
            case STOP:
                io.stop();
                break;
            case INTAKE:
                io.runVolts(volts:5);
                break;
            case OUTTAKE:
                io.runVolts(volts:-5);
                break;
        }]

    }
}