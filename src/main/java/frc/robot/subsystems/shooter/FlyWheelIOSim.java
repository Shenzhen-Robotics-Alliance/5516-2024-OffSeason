package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class FlyWheelIOSim implements FlyWheelIO {
    private final DCMotorSim dcMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.00064);
    @Override
    public void updateInputs(FlyWheelsInputs inputs) {
        dcMotorSim.update(Robot.defaultPeriodSecs);
        inputs.supplyCurrentAmps = dcMotorSim.getCurrentDrawAmps();
        inputs.flyWheelVelocityRevsPerSec = Units.radiansToRotations(
                dcMotorSim.getAngularVelocityRadPerSec()
        );
        inputs.flyWheelPositionRevs = Units.radiansToRotations(
                dcMotorSim.getAngularPositionRad()
        );
    }

    @Override
    public void runVoltage(double volts) {
        dcMotorSim.setInputVoltage(volts);
    }
}
