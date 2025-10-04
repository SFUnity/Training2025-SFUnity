package frc.robot.subsystems.Rollers;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

public class rollersiosim implements rollersio {
    private double appliedVolts = 0.0;

    public rollersiosim() {}

    @Override
    public void updateInputs (RollersioInputs inputs) {
        inputs.voltage = appliedVolts;
    }

    @Override
    public void runVolts (double volts) {
        appliedVolts = volts;
    }
}
