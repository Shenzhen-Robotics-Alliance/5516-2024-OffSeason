package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ChassisFaceToRotation extends Command {
    private final HolonomicDriveSubsystem driveSubsystem;
    private final Supplier<Rotation2d> targetRotationSupplier;
    private final PIDController chassisRotationController;
    private final Rotation2d tolerance;

    public ChassisFaceToRotation(HolonomicDriveSubsystem driveSubsystem, Supplier<Rotation2d> targetRotationSupplier, Rotation2d tolerance) {
        this.driveSubsystem = driveSubsystem;
        this.targetRotationSupplier = targetRotationSupplier;
        this.tolerance = tolerance;

        this.chassisRotationController = new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig);
    }

    @Override
    public void initialize() {
        chassisRotationController.calculate(
                driveSubsystem.getFacing().getRadians(),
                targetRotationSupplier.get().getRadians()
        );
    }

    @Override
    public void execute() {
        final double rotationFeedBack = chassisRotationController.calculate(driveSubsystem.getFacing().getRadians(), targetRotationSupplier.get().getRadians());
        driveSubsystem.runRobotCentricChassisSpeeds(new ChassisSpeeds(0, 0, rotationFeedBack));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        final double chassisAimingErrorDeg = driveSubsystem.getFacing().minus(targetRotationSupplier.get()).getDegrees();
        Logger.recordOutput("Shooter/chassisAimingErrorDeg", chassisAimingErrorDeg);
        final boolean ready = Math.abs(chassisAimingErrorDeg) < tolerance.getDegrees();
        Logger.recordOutput("Shooter/chassisAimingReady", ready);
        return ready;
    }

    public static ChassisFaceToRotation faceToTarget(HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier) {
        return new ChassisFaceToRotation(
                driveSubsystem,
                () -> targetPositionSupplier.get().minus(driveSubsystem.getPose().getTranslation()).getAngle(),
                Rotation2d.fromDegrees(4));
    }
}
