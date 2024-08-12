package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAndShootSequence extends SequentialCommandGroup  {
    public AimAndShootSequence(
            Pitch pitch, FlyWheels flyWheels, Intake intake,
            MapleShooterOptimization shooterOptimization,
            HolonomicDriveSubsystem drive,
            Supplier<Translation2d> targetPositionSupplier,
            BooleanSupplier externalShootCondition) {
        this(
                pitch, flyWheels, intake,
                shooterOptimization,
                drive,
                () -> drive.getPose().getTranslation(),
                targetPositionSupplier,
                externalShootCondition
        );
    }

    /**
     * creates a sequence to shoot at speaker in the following steps:
     * 1. runs a {@link PrepareToAim} command, until the shooter is prepared
     * 2. aims at the speaker continuously using a {@link AimAtSpeakerContinuously} command
     * 2. finally, when the shooters are ready and when the external shooting condition is met, execute the shoot
     * */
    public AimAndShootSequence(
            Pitch pitch, FlyWheels flyWheels, Intake intake,
            MapleShooterOptimization shooterOptimization,
            HolonomicDriveSubsystem drive,
            Supplier<Translation2d> robotScoringPositionSupplier,
            Supplier<Translation2d> targetPositionSupplier,
            BooleanSupplier externalShootCondition) {

        super();
        super.addRequirements(pitch, flyWheels, intake);
        super.addCommands(Commands.runOnce(intake::runIdle));

        super.addCommands(
                new PrepareToAim(flyWheels, pitch, shooterOptimization, robotScoringPositionSupplier, targetPositionSupplier)
                        .untilReady()
        );
        final AimAtSpeakerContinuously aimAtSpeakerContinuously = new AimAtSpeakerContinuously(
                flyWheels, pitch, shooterOptimization, drive, targetPositionSupplier
        );
        final Command waitForRightTimingAndShoot = Commands.waitUntil(aimAtSpeakerContinuously::readyToShoot)
                .andThen(Commands.waitUntil(externalShootCondition))
                .andThen(intake.executeLaunch());
        super.addCommands(aimAtSpeakerContinuously.alongWith(waitForRightTimingAndShoot));
    }
}
