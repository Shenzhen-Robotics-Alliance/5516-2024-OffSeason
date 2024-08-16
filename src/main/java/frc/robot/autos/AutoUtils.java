package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.AimAtSpeakerContinuously;
import frc.robot.commands.shooter.PrepareToAim;
import frc.robot.utils.MaplePathPlannerLoader;
import frc.robot.utils.MapleShooterOptimization;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoUtils {
    private final RobotContainer robot;
    private final AtomicReference<Optional<Rotation2d>> rotationTargetOverride;

    public AutoUtils(RobotContainer robot, AtomicReference<Optional<Rotation2d>> rotationTargetOverride) {
        this.robot = robot;
        this.rotationTargetOverride = rotationTargetOverride;
    }

    public Command executeGrabAndShootWithTimeOut(BooleanSupplier shootCondition, double timeOutSeconds) {
        return runFullIntake().until(robot.intake::isNotePresent)
                .andThen(Commands.waitUntil(shootCondition))
                .andThen(robot.intake.executeLaunch())
                .withTimeout(timeOutSeconds);
    }

    public Command runFullIntake() {
        return Commands.run(robot.intake::runFullIntakeVoltage, robot.intake);
    }

    public Command executeShootWithTimeOut(double delaySeconds, double timeOutSeconds) {
        return Commands.waitSeconds(delaySeconds)
                .andThen(robot.intake.executeLaunch())
                .withTimeout(timeOutSeconds);
    }

    public Command prepareToShootDuringFollowPathForSeconds(PathPlannerPath pathAtBlueAlliance, double untilTargetDistanceMeters, double timeOutSeconds) {
        final Supplier<Pose2d> endingPoseSupplier = MaplePathPlannerLoader.getEndingRobotPoseInCurrentAllianceSupplier(pathAtBlueAlliance);
        final BooleanSupplier closeEnough = () ->
                robot.drive.getPose().getTranslation()
                        .getDistance(endingPoseSupplier.get().getTranslation())
                < untilTargetDistanceMeters;
        final PrepareToAim prepareToAim = new PrepareToAim(
                robot.flyWheels, robot.pitch, robot.ledStatusLight, robot.shooterOptimization,
                () -> endingPoseSupplier.get().getTranslation(),
                Constants.CrescendoField2024Constants.SPEAKER_POSITION_SUPPLIER
        );
        return prepareToAim
                .untilReady()
                .until(closeEnough)
                .withTimeout(timeOutSeconds);
    }

    public Command aimAtSpeaker() {
        return aimAtSpeakerChassisOnly().alongWith(aimAtSpeakerShooterOnly());
    }

    public MapleShooterOptimization.ChassisAimAtSpeakerDuringAuto aimAtSpeakerChassisOnly() {
        return robot.shooterOptimization.chassisAimAtSpeakerDuringAuto(
                rotationTargetOverride,
                Constants.CrescendoField2024Constants.SPEAKER_POSITION_SUPPLIER,
                robot.drive
        );
    }
    public AimAtSpeakerContinuously aimAtSpeakerShooterOnly() {
        return new AimAtSpeakerContinuously(
                robot.flyWheels, robot.pitch, robot.ledStatusLight, robot.shooterOptimization, robot.drive,
                Constants.CrescendoField2024Constants.SPEAKER_POSITION_SUPPLIER,
                () -> true
        );
    }

    public Command runShooterIdle() {
        return Commands.run(() -> {
            robot.pitch.runSetPointProfiled(Math.toDegrees(30));
            robot.flyWheels.runRPMProfiled(0);
        }, robot.pitch, robot.flyWheels);
    }
}
