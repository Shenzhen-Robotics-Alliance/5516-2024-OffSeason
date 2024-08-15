package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public class StageSideEightNotes extends Auto {
    public StageSideEightNotes(RobotContainer robot) {
        super();

        super.addRequirements(robot.drive, robot.intake, robot.pitch, robot.flyWheels);

        super.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                "shoot first grab second normal"))
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
        return new Pose2d(1.36, 4.65, Rotation2d.fromDegrees(180));
    }
}
