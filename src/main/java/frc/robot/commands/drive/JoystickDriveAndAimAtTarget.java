package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JoystickDriveAndAimAtTarget extends Command {
    private final MapleJoystickDriveInput input;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final MapleShooterOptimization shooterOptimization;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final PIDController chassisRotationController;

    private final double pilotInputMultiplier;

    public JoystickDriveAndAimAtTarget(MapleJoystickDriveInput input, HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier, MapleShooterOptimization shooterOptimization, double pilotInputMultiplier) {
        this.targetPositionSupplier = targetPositionSupplier;
        this.shooterOptimization = shooterOptimization;
        this.pilotInputMultiplier = pilotInputMultiplier;
        this.chassisRotationController = new MaplePIDController(
                Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig
        );

        this.driveSubsystem = driveSubsystem;
        this.input = new MapleJoystickDriveInput(
                () -> input.joystickXSupplier.getAsDouble() * pilotInputMultiplier,
                () -> input.joystickYSupplier.getAsDouble() * pilotInputMultiplier,
                () -> 0
        );
    }

    @Override
    public void initialize() {
        this.chassisRotationController.calculate(driveSubsystem.getRawGyroYaw().getRadians());
        this.chassisRotationInPosition = false;
    }

    @Override
    public void execute() {
        final ChassisSpeeds pilotInputSpeeds = input.getJoystickChassisSpeeds(
                        driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(), driveSubsystem.getChassisMaxAngularVelocity())
                .times(pilotInputMultiplier),

                chassisSpeeds = pilotInputSpeeds.plus(new ChassisSpeeds(
                        0, 0,
                        getRotationalCorrectionVelocityRadPerSec()
                ));

        driveSubsystem.runDriverStationCentricChassisSpeeds(chassisSpeeds);
        super.execute();
    }

    public static double FEED_FORWARD_RATE = 1.3, ROTATION_TOLERANCE_DEGREES = 5;
    public double getRotationalCorrectionVelocityRadPerSec() {
        final Translation2d robotPosition = driveSubsystem.getPose().getTranslation();
        final ChassisSpeeds robotVelocityFieldRelative = driveSubsystem.getMeasuredChassisSpeedsFieldRelative();
        final Translation2d robotPositionAfterDt = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * Robot.defaultPeriodSecs,
                robotVelocityFieldRelative.vyMetersPerSecond * Robot.defaultPeriodSecs
        ));

        final Rotation2d targetedFacing =
                shooterOptimization.getShooterFacing(
                        targetPositionSupplier.get(),
                        robotPosition,
                        robotVelocityFieldRelative
                ),
                /* to calculate the derivative of target facing */
                targetedFacingAfterDT = shooterOptimization.getShooterFacing(
                        targetPositionSupplier.get(),
                        robotPositionAfterDt,
                        robotVelocityFieldRelative
                );
        final double targetedFacingChangeRateRadPerSec =
                targetedFacingAfterDT.minus(targetedFacing).getRadians()
                        / Robot.defaultPeriodSecs;
        Logger.recordOutput("Drive/Face To Target Rotation (Deg)", targetedFacing.getDegrees());

        final double feedBackRotationalSpeed = chassisRotationController.calculate(
                driveSubsystem.getFacing().getRadians(),
                targetedFacing.getRadians()),
                feedForwardRotationalSpeed = targetedFacingChangeRateRadPerSec
                        * FEED_FORWARD_RATE;

        final double chassisRotationalError = Math.abs(
                targetedFacing
                        .minus(driveSubsystem.getFacing())
                        .getDegrees()
        );
        Logger.recordOutput("Drive/Aim At Target Rational Error (Deg)", chassisRotationalError);
        this.chassisRotationInPosition = chassisRotationalError < ROTATION_TOLERANCE_DEGREES;
        SmartDashboard.putBoolean("Chassis Rotation Aiming Target Reached", chassisRotationInPosition);
        SmartDashboard.putNumber("Chassis Rotation Aiming Error", chassisRotationalError);

        return feedForwardRotationalSpeed + feedBackRotationalSpeed;
    }

    private boolean chassisRotationInPosition;
    public boolean chassisRotationInPosition() {
        return chassisRotationInPosition;
    }
}
