package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // private final IntakeIO io;
  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    //this.io = io;
  }

  @Override
  public void periodic() {
    //io.updateInputs(inputs);
    //Logger.processInputs("Intake/Inputs", inputs);
  }

  //public Command intake() {
    //return run(() -> io.runVolts(6.0));
  }

  //public Command eject() {
    //return run(() -> io.runVolts(-6.0));
  //}

  //public Command doStuff() {
    //return new SequentialCommandGroup(
        //intake().withTimeout(3), stop().withTimeout(1), eject().withTimeout(2));
  //}

  //public Command stop() {
    //return run(() -> io.runVolts(0.0));
  //}
//}
