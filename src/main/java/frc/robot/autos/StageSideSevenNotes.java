package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.AimAtSpeakerContinuously;
import frc.robot.utils.MapleShooterOptimization;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;


public class StageSideSevenNotes extends Auto {
    private AtomicReference<Optional<Rotation2d>> rotationTargetOverride = new AtomicReference<>(Optional.empty());
    public StageSideSevenNotes(RobotContainer robot) {
        super();
        super.addRequirements(robot.drive, robot.intake, robot.pitch, robot.flyWheels);
        PPHolonomicDriveController.setRotationTargetOverride(rotationTargetOverride::get);
        final AutoUtils utils = new AutoUtils(robot, rotationTargetOverride);

        /* shoot first */
        final PathPlannerPath moveToShootFirst = PathPlannerPath.fromPathFile("shoot first normal");
        final Command driveToShootFirstPose = utils.followPathAndStop(moveToShootFirst);
        final Command prepareToShoot1 = utils.prepareToShootDuringFollowPathForSeconds(
                moveToShootFirst,
                0.3, 0.5
        );
        final AimAtSpeakerContinuously aimAtSpeaker1 = utils.aimAtSpeakerShooterOnly();
        final MapleShooterOptimization.ChassisAimAtSpeakerDuringAuto aimAtSpeakerChassis1 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot1 = Commands.waitUntil(() -> aimAtSpeaker1.readyToShoot() && aimAtSpeakerChassis1.aimComplete())
                .andThen(Commands.run(robot.intake::runFullIntakeVoltage, robot.intake)
                        .until(() -> !robot.intake.isNotePresent())
                );
        final Command shootFirst = executeShoot1
                .deadlineWith(aimAtSpeaker1
                        .alongWith(aimAtSpeakerChassis1));
        super.addCommands(driveToShootFirstPose
                .alongWith(prepareToShoot1.andThen(shootFirst))
        );

        /* grab and shoot second */
        final Command driveToSecond = utils.followPathAndStop(PathPlannerPath.fromPathFile(
                "shoot second normal"
        ));
        final AimAtSpeakerContinuously aimAtSpeaker2 = utils.aimAtSpeakerShooterOnly();
        final MapleShooterOptimization.ChassisAimAtSpeakerDuringAuto aimAtSpeakerChassis2 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot2 = utils.executeGrabAndShootWithTimeOut(
                () -> aimAtSpeaker2.readyToShoot() && aimAtSpeakerChassis2.aimComplete(),
                4
        );
        final Command shootSecond = executeShoot2.deadlineWith(
                aimAtSpeaker2.alongWith(aimAtSpeakerChassis2)
        );
        super.addCommands(driveToSecond
                .alongWith(shootSecond)
        );
        super.addCommands(Commands.runOnce(robot.intake::runIdle, robot.intake));

        /* grab and shoot third */
        final PathPlannerPath grabThirdAndMoveToShootingPose = PathPlannerPath.fromPathFile("grab third and shoot normal");
        final Command intakeThirdUntilTouchingNote = Commands.run(robot.intake::runFullIntakeVoltage, robot.intake)
                .until(robot.intake::isNoteTouchingIntake)
                .deadlineWith(utils.runShooterIdle())
                .withTimeout(2);
        final Command prepareToShootThird = utils.prepareToShootDuringFollowPathForSeconds(
                grabThirdAndMoveToShootingPose,
                0.3, 0.8
        );
        final Command prepareToShootThirdAndPushNoteForward = prepareToShootThird.deadlineWith(robot.intake.executeIntakeNote());
        final AimAtSpeakerContinuously aimAtSpeaker3 = utils.aimAtSpeakerShooterOnly();
        final MapleShooterOptimization.ChassisAimAtSpeakerDuringAuto aimAtSpeakerChassis3 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot3 = Commands.waitUntil(() -> aimAtSpeaker3.readyToShoot() && aimAtSpeakerChassis3.aimComplete())
                .deadlineWith(aimAtSpeaker3.alongWith(aimAtSpeakerChassis3))
                .andThen(Commands.run(robot.intake::runFullIntakeVoltage, robot.intake)
                        .until(() -> !robot.intake.isNotePresent())
                );
        final Command grabThirdNoteAndShoot = intakeThirdUntilTouchingNote
                .andThen(prepareToShootThirdAndPushNoteForward)
                .andThen(executeShoot3);
        super.addCommands(utils.followPathAndStop(grabThirdAndMoveToShootingPose)
                .alongWith(grabThirdNoteAndShoot)
        );

        /* shoot fourth */
        final Command driveToFourth = utils.followPathAndStop(PathPlannerPath.fromPathFile(
                "shoot fourth normal"
        ));
        final AimAtSpeakerContinuously aimAtSpeaker4 = utils.aimAtSpeakerShooterOnly();
        final MapleShooterOptimization.ChassisAimAtSpeakerDuringAuto aimAtSpeakerChassis4 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot4 = utils.executeGrabAndShootWithTimeOut(
                () -> aimAtSpeaker4.readyToShoot() && aimAtSpeakerChassis4.aimComplete(),
                2
        );
        final Command shootFourth= executeShoot4.deadlineWith(
                aimAtSpeaker4.alongWith(aimAtSpeakerChassis4)
        );
        super.addCommands(driveToFourth
                .alongWith(shootFourth)
        );
        super.addCommands(Commands.runOnce(robot.intake::runIdle, robot.intake));

        if (true) return;

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot fifth normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "grab sixth normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot sixth normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "grab seventh normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot seventh normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "grab eighth normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot eighth normal"
        )));
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.46, 4.13, Rotation2d.fromDegrees(180));
    }
}
