package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class AmpSideSixNotesFast extends Auto {
    private AtomicReference<Optional<Rotation2d>> rotationTargetOverride = new AtomicReference<>(Optional.empty());
    public AmpSideSixNotesFast(RobotContainer robot) {
        final AutoUtils utils = new AutoUtils(robot, rotationTargetOverride);
        super.addRequirements(robot.drive, robot.intake, robot.pitch, robot.flyWheels);
        PPHolonomicDriveController.setRotationTargetOverride(rotationTargetOverride::get);


    }
    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.47, 6.32, Rotation2d.fromDegrees(180));
    }
}
