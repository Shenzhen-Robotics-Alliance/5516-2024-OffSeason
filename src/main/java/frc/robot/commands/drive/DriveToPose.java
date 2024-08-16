package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import frc.robot.utils.MechanismControl.MapleProfiledPIDController;

import java.util.function.Supplier;

public class DriveToPose extends Command {
    private final Supplier<Pose2d> desiredPoseSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final HolonomicDriveController positionController;
    private double speedConstrainMPS = 3;

    public DriveToPose(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> desiredPoseSupplier, Pose2d tolerance, double speedConstrainMPS) {
        this(driveSubsystem, desiredPoseSupplier);
        this.positionController.setTolerance(tolerance);
        this.speedConstrainMPS = speedConstrainMPS;
    }
    public DriveToPose(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> desiredPoseSupplier) {
        this.desiredPoseSupplier = desiredPoseSupplier;
        this.driveSubsystem = driveSubsystem;
        this.positionController = createPositionController();

        super.addRequirements(driveSubsystem);
    }


    @Override
    public void initialize() {
        getFeedBackSpeeds();
    }

    @Override
    public void execute() {
        ChassisSpeeds feedBackSpeeds = getFeedBackSpeeds();
        final double feedBackSpeedMagnitude = Math.hypot(feedBackSpeeds.vxMetersPerSecond, feedBackSpeeds.vyMetersPerSecond);
        if (feedBackSpeedMagnitude < speedConstrainMPS)
            feedBackSpeeds = feedBackSpeeds.times(speedConstrainMPS / feedBackSpeedMagnitude);
        driveSubsystem.runRobotCentricChassisSpeeds(feedBackSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    /**
     * @return the feed-back speed, robot-relative
     * */
    private ChassisSpeeds getFeedBackSpeeds() {
        return positionController.calculate(driveSubsystem.getPose(), desiredPoseSupplier.get(), 0, desiredPoseSupplier.get().getRotation());
    }

    @Override
    public boolean isFinished() {
        return this.positionController.atReference();
    }

    public static HolonomicDriveController createPositionController() {
        return new HolonomicDriveController(
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MapleProfiledPIDController(Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig, Constants.SwerveDriveChassisConfigs.chassisRotationalConstraints)
        );
    }
}
