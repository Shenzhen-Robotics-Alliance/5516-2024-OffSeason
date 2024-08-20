package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;

public class FeedShot {
    public static Command prepareToFeedForever(Pitch pitch, FlyWheels flyWheels, Intake intake) {
        final Command preparePitch = Commands.run(() -> pitch.runSetPointProfiled(Constants.PitchConfigs.PITCH_LOWEST_ROTATION_RAD), pitch),
                prepareFlyWheels = Commands.run(() -> flyWheels.runRPMProfiled(2500), flyWheels),
                runIntakeIdle = Commands.run(intake::runIdle, intake);


        return preparePitch.alongWith(prepareFlyWheels).alongWith(runIntakeIdle);
    }

    public static Command prepareToFeedUntilReady(Pitch pitch, FlyWheels flyWheels, Intake intake) {
        return prepareToFeedForever(pitch, flyWheels, intake).until(
                () -> pitch.inPosition()
                && flyWheels.flyWheelsReady()
        );
    }

    public static Command shootFeed(Pitch pitch, FlyWheels flyWheels, Intake intake) {
        return prepareToFeedUntilReady(pitch, flyWheels, intake).andThen(
                intake.executeIntakeNote()
                        .deadlineWith(prepareToFeedForever(pitch, flyWheels, intake))
        );
    }
}
