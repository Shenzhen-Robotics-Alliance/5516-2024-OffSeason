package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;

public class PrepareToAmp extends Command {
    private final Pitch pitch;
    private final FlyWheels flyWheels;
    public PrepareToAmp(Pitch pitch, FlyWheels flyWheels) {
        super();
        this.pitch = pitch;
        this.flyWheels = flyWheels;
        super.addRequirements(pitch, flyWheels);
    }

    private boolean running = false;
    @Override
    public void initialize() {
        running = false;
    }

    @Override
    public void execute() {
        running = true;
        pitch.runSetPointProfiled(Math.toRadians(65));
        flyWheels.runRPMProfiled(500);
    }

    public boolean isReady() {
        return running && pitch.inPosition() && flyWheels.flyWheelsReady();
    }

    public Command untilReady() {
        return this.until(this::isReady);
    }
}
