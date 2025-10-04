package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command intake() {
    return run(() -> io.runVolts(10));
  }

  public Command stop() {
    return run(() -> io.runVolts(0.0));
  }
  public Command eject() {
    return run(() -> io.runVolts(-6.7));
  }
}
