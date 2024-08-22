package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;

public class FeedShot {
    public static Command prepareToFeedForever(Pitch pitch, FlyWheels flyWheels) {
        final Command preparePitch = Commands.run(() -> pitch.runSetPointProfiled(Constants.PitchConfigs.PITCH_LOWEST_ROTATION_RAD), pitch),
                prepareFlyWheels = Commands.run(() -> flyWheels.runRPMProfiled(3500), flyWheels);
        return preparePitch.alongWith(prepareFlyWheels);
    }

    public static Command prepareToFeedUntilReady(Pitch pitch, FlyWheels flyWheels) {
        return prepareToFeedForever(pitch, flyWheels).until(
                () -> pitch.inPosition()
                && flyWheels.flyWheelsReady()
        );
    }

    public static Command shootFeed(Pitch pitch, FlyWheels flyWheels, Intake intake) {
        return prepareToFeedUntilReady(pitch, flyWheels).andThen(
                intake.executeLaunch()
                        .withTimeout(0.5)
                        .deadlineWith(prepareToFeedForever(pitch, flyWheels))
        );
    }
}
