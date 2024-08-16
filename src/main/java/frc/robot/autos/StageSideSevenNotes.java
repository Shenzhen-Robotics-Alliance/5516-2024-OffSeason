package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;


public class StageSideSevenNotes extends Auto {
    private AtomicReference<Optional<Rotation2d>> rotationTargetOverride = new AtomicReference<>(Optional.empty());
    public StageSideSevenNotes(RobotContainer robot) {
        super();
        super.addRequirements(robot.drive, robot.intake, robot.pitch, robot.flyWheels);


    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.46, 4.13, Rotation2d.fromDegrees(180));
    }
}
