package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.shooter.ShooterVisualizer;
import frc.robot.utils.Alert;
import org.littletonrobotics.junction.Logger;

public class Intake extends MapleSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    private boolean lowerBeamBrakeAlwaysTrue, upperBeamBrakeAlwaysTrue;
    private final Alert lowerBeamBrakeAlwaysBlockedAlert, upperBeamBrakeAlwaysBlockedAlert;
    public Intake(IntakeIO intakeIO) {
        super("Intake");
        this.io = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();

        this.lowerBeamBrakeAlwaysTrue = this.upperBeamBrakeAlwaysTrue = true;
        this.lowerBeamBrakeAlwaysBlockedAlert = new Alert("Intake LOWER Beam Breaker Always Blocked", Alert.AlertType.WARNING);
        this.upperBeamBrakeAlwaysBlockedAlert = new Alert("Intake UPPER Beam Breaker Always Blocked", Alert.AlertType.WARNING);

        super.setDefaultCommand(Commands.run(this::runIdle, this));
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        this.lowerBeamBrakeAlwaysBlockedAlert.setActivated(
                lowerBeamBrakeAlwaysTrue &= inputs.lowerBeamBreakBlocked
        );
        this.upperBeamBrakeAlwaysBlockedAlert.setActivated(
                upperBeamBrakeAlwaysTrue &= inputs.upperBeamBreakerBlocked
        );

        visualizeNoteInShooter();
    }

    private void visualizeNoteInShooter() {
        final ShooterVisualizer.NotePositionInShooter notePositionInShooter;
        if (inputs.upperBeamBreakerBlocked)
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.AT_TOP;
        else if (inputs.lowerBeamBreakBlocked)
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.AT_BOTTOM;
        else
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.GONE;
        ShooterVisualizer.setNoteInShooter(notePositionInShooter);
    }

    public void runFullIntakeVoltage() {
        io.runIntakeVoltage(12);
    }

    public void runMinimumPropellingVoltage() {
        io.runIntakeVoltage(2);
    }

    public boolean isNotePresent() {
        return inputs.upperBeamBreakerBlocked;
    }

    public Command executeIntakeNote() {
        return Commands.run(() ->{
            if (inputs.lowerBeamBreakBlocked)
                runMinimumPropellingVoltage();
            else
                runFullIntakeVoltage();
        }, this)
                .until(this::isNotePresent)
                .onlyIf(() -> !this.isNotePresent());
    }

    public Command executeLaunch() {
        return Commands.run(this::runFullIntakeVoltage, this)
                .until(() -> !isNotePresent());
    }

    public void runIdle() {
        io.runIntakeVoltage(0);
    }
}

