package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.Alert;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.PitchConfigs.*;
public class Pitch extends MapleSubsystem {
    private final PitchIO io;
    private final PitchInputsAutoLogged inputs;

    private double setPointRad = PITCH_LOWEST_ROTATION_RAD;
    private final Alert notCalibratedAlert = new Alert("Pitch not calibrated!", Alert.AlertType.ERROR);

    private final ArmFeedforward feedForward;
    private final PIDController feedBack;
    private final TrapezoidProfile profile;
    public Pitch(PitchIO io) {
        super("Pitch");
        this.io = io;
        this.inputs = new PitchInputsAutoLogged();

        this.feedForward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
        this.feedBack = new MaplePIDController(PITCH_PID);
        this.profile = new TrapezoidProfile(PITCH_PROFILE_CONSTRAIN);

        notCalibratedAlert.setActivated(false);

        setDefaultCommand(Commands.run(() -> runSetPointProfiled(PITCH_LOWEST_ROTATION_RAD), this));
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs("Pitch", inputs);

        notCalibratedAlert.setActivated(!inputs.calibrated);
        ShooterVisualizer.setPitchAngle(inputs.pitchAngleRad);
    }

    private double previousStateVelocity = 0;
    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(PITCH_LOWEST_ROTATION_RAD, 0);
    public void runSetPointProfiled(double setPointRad) {
        if (checkSetPointOutOfRange(setPointRad) || !inputs.calibrated) {
            io.runPitchVoltage(0);
            return;
        }

        this.setPointRad = setPointRad;
        this.currentState = profile.calculate(
                Robot.defaultPeriodSecs,
                currentState,
                new TrapezoidProfile.State(setPointRad, 0)
        );
        final double stateAcceleration = previousStateVelocity / Robot.defaultPeriodSecs;
        runControlLoops(currentState.position, currentState.velocity, stateAcceleration);
        this.previousStateVelocity = currentState.velocity;
    }

    public void forceSetPoint(double setPointRad, double velocitySetPointRadPerSec) {
        if (checkSetPointOutOfRange(setPointRad) || !inputs.calibrated) {
            io.runPitchVoltage(0);
            return;
        }

        this.setPointRad = setPointRad;
        this.currentState = new TrapezoidProfile.State(setPointRad, velocitySetPointRadPerSec);
        this.previousStateVelocity = velocitySetPointRadPerSec;
        runControlLoops(setPointRad, velocitySetPointRadPerSec, 0);
    }

    private boolean checkSetPointOutOfRange(double setPointRad) {
        if (setPointRad < PITCH_LOWEST_ROTATION_RAD || setPointRad > PITCH_HIGHER_LIMIT_RAD) {
            DriverStation.reportError("attempt to run a pitch setpoint out of range: " + setPointRad, true);
            return true;
        }
        return false;
    }

    private void runControlLoops(double currentPositionSetPointRad, double currentVelocitySetPointRadPerSec, double currentAcceleration) {
        final double
                feedForwardVoltage = feedForward.calculate(
                        inputs.pitchAngleRad, currentVelocitySetPointRadPerSec, currentAcceleration
                ),
                feedBackVoltage = feedBack.calculate(
                        inputs.pitchAngleRad, currentPositionSetPointRad
                );
        final double
                safetyConstrainLow = inputs.pitchAngleRad <= PITCH_LOWEST_ROTATION_RAD ? 0 : -10,
                safetyConstrainHigh = inputs.pitchAngleRad > PITCH_HIGHER_LIMIT_RAD ? 0 : 12,
                pitchVoltage = MathUtil.clamp(
                        feedForwardVoltage + feedBackVoltage, safetyConstrainLow, safetyConstrainHigh
                );

        Logger.recordOutput("Shooter/Pitch Actual Position (Deg)", Math.toDegrees(inputs.pitchAngleRad));
        Logger.recordOutput("Shooter/Pitch Control Voltage", pitchVoltage);
        io.runPitchVoltage(pitchVoltage);
    }

    public boolean inPosition() {
        return Math.abs(inputs.pitchAngleRad - setPointRad) < PITCH_PID.errorTolerance;
    }
}