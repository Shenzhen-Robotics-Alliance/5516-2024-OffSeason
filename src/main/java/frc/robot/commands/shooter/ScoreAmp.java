package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;

public class ScoreAmp extends Command {
    private final Intake intake;
    private final Pitch pitch;
    private final FlyWheels flyWheels;
    public ScoreAmp(Intake intake, Pitch pitch, FlyWheels flyWheels) {
        super();
        this.intake = intake;
        this.pitch = pitch;
        this.flyWheels = flyWheels;
        super.addRequirements(intake, pitch, flyWheels);
    }

    @Override
    public void execute() {
        intake.runFullIntakeVoltage();
        pitch.runSetPointProfiled(Math.toRadians(92));
        flyWheels.runRPMProfiled(500);
    }

    @Override
    public boolean isFinished() {
        return !intake.isNotePresent();
    }
}
