package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.PitchConfigs.*;

public class PitchIOReal implements PitchIO {
    private final TalonFX pitchFalcon1 = new TalonFX(17), pitchFalcon2 = new TalonFX(18);
    private final StatusSignal<Double> pitchMotor1SuppliedAmps, pitchMotor2SuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec;
    private final DigitalInput limitSwitch = new DigitalInput(0);

    private double encoderPositionAtLowestPoint;
    public PitchIOReal() {
        pitchFalcon1.setInverted(true);
        pitchMotor1SuppliedAmps = pitchFalcon1.getSupplyCurrent();
        pitchMotor2SuppliedAmps = pitchFalcon2.getSupplyCurrent();
        pitchRelativeEncoderPositionRev = pitchFalcon1.getPosition();
        pitchEncoderVelocityRevPerSec = pitchFalcon1.getVelocity();
        encoderPositionAtLowestPoint = pitchRelativeEncoderPositionRev.getValue();
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pitchMotor1SuppliedAmps, pitchMotor2SuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec
        );

        final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 20;
        pitchFalcon1.getConfigurator().apply(currentLimitsConfigs);
        pitchFalcon2.getConfigurator().apply(currentLimitsConfigs);
        pitchFalcon1.optimizeBusUtilization();
        pitchFalcon2.optimizeBusUtilization();
    }
    @Override
    public void updateInputs(PitchInputs pitchInputs) {
        BaseStatusSignal.refreshAll(
                pitchMotor1SuppliedAmps, pitchMotor2SuppliedAmps, pitchRelativeEncoderPositionRev, pitchEncoderVelocityRevPerSec
        );

        pitchInputs.calibrated |= isLimitSwitchPressed();
        Logger.recordOutput("Shooter/Pitch Limit", isLimitSwitchPressed());
        if (isLimitSwitchPressed())
            encoderPositionAtLowestPoint = pitchRelativeEncoderPositionRev.getValue();

        pitchInputs.pitchSuppliedCurrentAmps = pitchMotor1SuppliedAmps.getValue() + pitchMotor2SuppliedAmps.getValue();
        pitchInputs.pitchAngularVelocityRadPerSec = Units.rotationsToRadians(
                pitchEncoderVelocityRevPerSec.getValue() / GEAR_RATIO
        );
        pitchInputs.pitchAngleRad = Units.rotationsToRadians(
                (pitchRelativeEncoderPositionRev.getValue() - encoderPositionAtLowestPoint)
                        / GEAR_RATIO
        ) + PITCH_LOWEST_ROTATION_RAD;
    }

    @Override
    public void runPitchVoltage(double volts) {
        final VoltageOut voltageOut = new VoltageOut(volts).withEnableFOC(true);
        pitchFalcon1.setControl(voltageOut);
        pitchFalcon2.setControl(voltageOut);
    }

    @Override
    public void setPitchLock(boolean enabled) {
        final NeutralModeValue neutralModeValue = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        pitchFalcon1.setNeutralMode(neutralModeValue);
        pitchFalcon2.setNeutralMode(neutralModeValue);
    }

    private boolean isLimitSwitchPressed() {
        // if the sensor is a 3-wire sensor, this should be limitSwitch.get()
        // if the sensor is a simple sensor, this should be !limitSwitch.get
        return !limitSwitch.get();
    }
}
