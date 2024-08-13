package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.FlyWheels;
import frc.robot.subsystems.shooter.Pitch;
import frc.robot.utils.MapleShooterOptimization;

import java.util.function.Supplier;

public class DriveToPoseAndShootSequence extends AutoAlignment {
    public DriveToPoseAndShootSequence(
            Intake intake, Pitch pitch, FlyWheels flyWheels,
            MapleShooterOptimization shooterOptimization,
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Translation2d> robotShootingPositionSupplier,
            Supplier<Translation2d> speaerPositionSupplier
    ) {
        super(
                driveSubsystem,
                () -> {
                    final Translation2d displacementToTarget = speaerPositionSupplier.get().minus(robotShootingPositionSupplier.get());
                    return new Pose2d(
                            robotShootingPositionSupplier.get(),
                            displacementToTarget.getAngle()
                    );
                },
                () -> new Pose2d(
                        robotShootingPositionSupplier.get(),
                        shooterOptimization.getShooterFacing(
                                speaerPositionSupplier.get(),
                                driveSubsystem.getPose().getTranslation(),
                                driveSubsystem.getMeasuredChassisSpeedsFieldRelative())
                ),
                new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(3)),
                0.75,
                new PrepareToAim(flyWheels, pitch, shooterOptimization, robotShootingPositionSupplier, speaerPositionSupplier),
                new AimAndShootSequence(
                        pitch, flyWheels, intake, shooterOptimization, driveSubsystem,
                        robotShootingPositionSupplier,
                        speaerPositionSupplier,
                        () -> true
                )
        );

        super.addRequirements(driveSubsystem, pitch, flyWheels, intake);
    }
}
