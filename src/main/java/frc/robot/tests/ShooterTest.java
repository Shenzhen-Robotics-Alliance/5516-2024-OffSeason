package frc.robot.tests;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ShooterTest extends Command {
    private final TalonFX
            shooter1 = new TalonFX(21), shooter2 = new TalonFX(22),
            intake = new TalonFX(16),
            pitch1 = new TalonFX(17), pitch2 = new TalonFX(18);
    private double pitch1ZeroPosition;
    public ShooterTest() {
        intake.setInverted(true);

        shooter1.setInverted(true);
        shooter2.setInverted(true);

        pitch1.setNeutralMode(NeutralModeValue.Coast);
        pitch2.setNeutralMode(NeutralModeValue.Coast);
        pitch1.setInverted(true);
    }

    private final XboxController xboxController = new XboxController(1);
    @Override
    public void execute() {
        final VoltageOut shooterOut = new VoltageOut(-xboxController.getLeftY() * 12).withEnableFOC(false),
                intakeOut = new VoltageOut(-xboxController.getRightY() * 12).withEnableFOC(true);
        shooter1.setControl(shooterOut);
        shooter2.setControl(shooterOut);
        intake.setControl(intakeOut);

        Logger.recordOutput("Test/Shooter RPM", shooter1.getVelocity().getValue() * 60);
        Logger.recordOutput("Test/Shooter Current", shooter1.getSupplyCurrent().getValue() + shooter1.getSupplyCurrent().getValue());

        Logger.recordOutput("Test/Pitch1 Encoder", (pitch1.getPosition().getValue() - pitch1ZeroPosition) / 166.66 * 360);
    }
}
