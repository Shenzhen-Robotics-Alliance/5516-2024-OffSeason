package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeFalcon = new TalonFX(16);
    private final DigitalInput lowerBeamBreaker = new DigitalInput(2), upperBeamBreaker = new DigitalInput(1);
    private final StatusSignal<Double> intakeCurrent;
    public IntakeIOReal() {
        this.intakeCurrent = intakeFalcon.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeCurrent);
        intakeFalcon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(intakeCurrent);
        inputs.motorCurrent = intakeCurrent.getValue();
        inputs.lowerBeamBreakBlocked = !lowerBeamBreaker.get();
        inputs.upperBeamBreakerBlocked = !upperBeamBreaker.get();
    }

    @Override
    public void runIntakeVoltage(double volts) {
        intakeFalcon.setControl(new VoltageOut(volts).withEnableFOC(true));
    }
}
