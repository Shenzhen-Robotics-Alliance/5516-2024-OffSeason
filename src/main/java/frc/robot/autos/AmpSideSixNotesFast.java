package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AimAtSpeakerFactory;
import frc.robot.commands.CommandOnFly;
import frc.robot.commands.shooter.FollowPathGrabAndShootStill;


public class AmpSideSixNotesFast extends Auto {
    public AmpSideSixNotesFast(RobotContainer robot) {
        super();
        final boolean[] noteFightFailed = new boolean[2];

        /* shoot preloaded */
        super.addCommands(AimAtSpeakerFactory.shootAtSpeakerStill(robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight));
        super.addCommands(AutoUtils.setIdleForSuperStructureCommand(robot));

        /* grab second */
        final Command chassisMoveToSecond = AutoBuilder.followPath(PathPlannerPath.fromPathFile("move to second and grab fast"))
                .andThen(Commands.runOnce(robot.drive::stop, robot.drive));
        final Command grabSecondWhenClose = Commands.waitSeconds(1.5).andThen(
                robot.intake.suckNoteUntilTouching().withTimeout(2)
                        .finallyDo(() -> noteFightFailed[0] = !robot.intake.isNotePresent())
        );
        super.addCommands(grabSecondWhenClose.deadlineWith(
                chassisMoveToSecond.alongWith(robot.pitch.getPitchDefaultCommand())
        ));

        /* retry grab second */
        final Command chassisMoveToRetrySecond = AutoBuilder.followPath(PathPlannerPath.fromPathFile("retry grab second fast"))
                .andThen(Commands.runOnce(robot.drive::stop, robot.drive));
        final Command retryGrabSecond = Commands.waitSeconds(0.5).onlyIf(() -> noteFightFailed[0]).andThen(
                robot.intake.suckNoteUntilTouching().withTimeout(2)
        );
        super.addCommands(retryGrabSecond
                .deadlineWith(chassisMoveToRetrySecond)
        );

        /* move and shoot second */
        super.addCommands(new FollowPathGrabAndShootStill(
                PathPlannerPath.fromPathFile("shoot second fast"),
                1.5,
                robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight
        ));
        super.addCommands(AutoUtils.setIdleForSuperStructureCommand(robot));


        /* move to third and grab */
        final Command moveToThirdNormal = AutoBuilder.followPath(PathPlannerPath.fromPathFile("move to third fast"))
                .andThen(Commands.runOnce(robot.drive::stop, robot.drive));
        final Command moveToThirdAlter = AutoBuilder.followPath(PathPlannerPath.fromPathFile("move to third fast alter"))
                .andThen(Commands.runOnce(robot.drive::stop, robot.drive));
        final Command moveToThirdDecisive = new CommandOnFly(() -> noteFightFailed[0] ? moveToThirdAlter : moveToThirdNormal)
                .deadlineWith(robot.pitch.getPitchDefaultCommand());
        final Command intakeThird = Commands.waitSeconds(1)
                .andThen(robot.intake.suckNoteUntilTouching().withTimeout(4));
        super.addCommands(intakeThird.deadlineWith(moveToThirdDecisive));

        /* shoot third */
        super.addCommands(new FollowPathGrabAndShootStill(
                PathPlannerPath.fromPathFile("move to shoot third fast"),
                1.2,
                robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight
        ));

        /* grab fourth and shoot */
        super.addCommands(new FollowPathGrabAndShootStill(
                PathPlannerPath.fromPathFile("grab fourth and shoot fast"),
                0.8,
                robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight
        ));

        /* grab fifth and shoot */
        super.addCommands(new FollowPathGrabAndShootStill(
                PathPlannerPath.fromPathFile("grab fifth and shoot fast"),
                0.8,
                robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight
        ));

        /* grab sixth and shoot */
        super.addCommands(new FollowPathGrabAndShootStill(
                PathPlannerPath.fromPathFile("grab sixth and shoot fast"),
                0.8,
                robot.drive, robot.intake, robot.pitch, robot.flyWheels, robot.shooterOptimization, robot.ledStatusLight
        ));
    }
    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.47, 6.32, Rotation2d.fromDegrees(-150));
    }
}
