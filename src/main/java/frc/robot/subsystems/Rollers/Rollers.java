package frc.robot.subsystems.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final rollersio io;
  private final RollersioInputsAutoLogged inputs = new RollersioInputsAutoLogged();

  public Rollers(rollersio io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command intake() {
    return run(() -> io.runVolts(-6.0));
  }

  public Command stop() {
    return run(() -> io.runVolts(0.0));
  }
}
