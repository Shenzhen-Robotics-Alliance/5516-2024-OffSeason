package frc.robot.subsystems.intake;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.ShooterVisualizer;
import frc.robot.utils.Alert;
import frc.robot.utils.LEDAnimation;
import org.littletonrobotics.junction.Logger;

public class Intake extends MapleSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    private static final LEDAnimation RUNNING = new LEDAnimation.Charging(255, 255, 255, 2), // orange charging
            GRABBED_NOTE = new LEDAnimation.ShowColor(230, 255, 0); // yellow

    private boolean lowerBeamBrakeAlwaysTrue, upperBeamBrakeAlwaysTrue;
    private final Alert lowerBeamBrakeAlwaysBlockedAlert, upperBeamBrakeAlwaysBlockedAlert;
    private final BooleanConsumer noteInShooterConsumer;
    public Intake(IntakeIO intakeIO, BooleanConsumer noteInShooterConsumer) {
        super("Intake");
        this.io = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();

        this.lowerBeamBrakeAlwaysTrue = this.upperBeamBrakeAlwaysTrue = true;
        this.lowerBeamBrakeAlwaysBlockedAlert = new Alert("Intake LOWER Beam Breaker Always Blocked", Alert.AlertType.WARNING);
        this.upperBeamBrakeAlwaysBlockedAlert = new Alert("Intake UPPER Beam Breaker Always Blocked", Alert.AlertType.WARNING);

        this.noteInShooterConsumer = noteInShooterConsumer;

        super.setDefaultCommand(Commands.run(this::runIdle, this));
    }

    @Override
    public void onDisable() {
        runIdle();
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
        noteInShooterConsumer.accept(isNotePresent());

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

    public void runInvertVoltage() {
        io.runIntakeVoltage(-12);
    }

    public void runMinimumPropellingVoltage() {
        io.runIntakeVoltage(4);
    }

    public boolean isNotePresent() {
        return inputs.upperBeamBreakerBlocked || inputs.lowerBeamBreakBlocked;
    }

    public boolean isNoteInUpperPosition() {
        return inputs.upperBeamBreakerBlocked;
    }

    public Command executeIntakeNote(JoystickDrive joystickDrive) {
        return Commands.run(() -> {
                    if (inputs.lowerBeamBreakBlocked) {
                        runMinimumPropellingVoltage();
                        joystickDrive.resetSensitivity();
                    }
                    else {
                        runFullIntakeVoltage();
                        joystickDrive.setSensitivity(0.4, 0.4);
                    }
                }, this)
                .until(() -> inputs.upperBeamBreakerBlocked)
                .onlyIf(() -> !inputs.upperBeamBreakerBlocked)
                .andThen(joystickDrive::resetSensitivity)
                .finallyDo(this::runIdle);
    }

    public Command executeIntakeNote(JoystickDrive joystickDrive, LEDStatusLight statusLight) {
        final Command executeIntake =  executeIntakeNote(joystickDrive)
                .raceWith(Commands.run(() -> statusLight.setAnimation(RUNNING), statusLight))
                .andThen(statusLight.playAnimationAndStop(GRABBED_NOTE, 1.5));
        return executeIntake;
    }

    public Command executeLaunch() {
        return Commands.run(this::runFullIntakeVoltage, this)
                .onlyIf(this::isNotePresent)
                .until(() -> !isNotePresent())
                .finallyDo(this::runIdle);
    }

    public void runIdle() {
        io.runIntakeVoltage(0);
    }
}

