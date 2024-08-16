package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;


public class AmpSideSixNotesFast extends Auto {
    public AmpSideSixNotesFast(RobotContainer robot) {
        super.addRequirements(robot.drive, robot.intake, robot.pitch, robot.flyWheels);
    }
    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.47, 6.32, Rotation2d.fromDegrees(180));
    }
}
