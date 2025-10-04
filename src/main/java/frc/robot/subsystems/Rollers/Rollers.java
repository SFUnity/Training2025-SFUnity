package frc.robot.subsystems.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final rollersio io;
  private final rollersioInputsAutoLogged inputs = new rollersioInputsAutoLogged();

  public Rollers(rollersio io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command command() {
    return run(() -> method());
  }
}
