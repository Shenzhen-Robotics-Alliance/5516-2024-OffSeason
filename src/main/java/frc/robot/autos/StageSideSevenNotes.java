package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.shooter.AimAtSpeakerContinuously;
import frc.robot.utils.MaplePathPlannerLoader;

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
        final Command driveToShootFirstPose = AutoBuilder.followPath(moveToShootFirst);
        final Command prepareToShoot1 = utils.prepareToShootDuringFollowPathForSeconds(
                moveToShootFirst,
                0.3, 0.5
        );
        final AimAtSpeakerContinuously aimAtSpeaker1 = utils.aimAtSpeakerShooterOnly();
        final Command aimAtSpeakerChassis1 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot1 = Commands.waitUntil(aimAtSpeaker1::readyToShoot)
                .andThen(robot.intake.executeLaunch());
        final Command shootFirst = executeShoot1
                .deadlineWith(aimAtSpeaker1
                        .alongWith(aimAtSpeakerChassis1));
        super.addCommands(driveToShootFirstPose
                .alongWith(prepareToShoot1.andThen(shootFirst))
        );

        /* grab and shoot second */
        final Command driveToSecond = AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot second normal"
        ));
        final AimAtSpeakerContinuously aimAtSpeaker2 = utils.aimAtSpeakerShooterOnly();
        final Command aimAtSpeakerChassis2 = utils.aimAtSpeakerChassisOnly();
        final Command executeShoot2 = utils.executeGrabAndShootWithTimeOut(aimAtSpeaker2::readyToShoot, 1);
        final Command shootSecond = executeShoot2.deadlineWith(
                aimAtSpeaker2.alongWith(aimAtSpeakerChassis2)
        );
        super.addCommands(driveToSecond
                .alongWith(shootSecond)
        );

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot third and fourth normal"
        )));

        super.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "grab fifth normal"
        )));

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
