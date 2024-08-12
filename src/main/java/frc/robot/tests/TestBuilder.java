package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

public class TestBuilder {
    public static SendableChooser<Supplier<Command>> buildTestsChooser(RobotContainer robotContainer) {
        final SendableChooser<Supplier<Command>> testsChooser = new SendableChooser<>();
        testsChooser.setDefaultOption("None", Commands::none);
        testsChooser.addOption("Wheels Calibration", WheelsCalibrationCTRE::new);
        testsChooser.addOption("Field Display Test", FieldDisplayTest::new);
        testsChooser.addOption("Robot Simulation Test", PhysicsSimulationTest::new);
        testsChooser.addOption("LED Test", () -> new LEDTest(robotContainer.ledStatusLight));
        testsChooser.addOption("Shooter Test", ShooterTest::new);

        testsChooser.addOption(
                "FlyWheel0 SysId Dynamic (Forward)",
                () -> robotContainer.flyWheels.sysIdRoutines[0].dynamic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "FlyWheel0 SysId Dynamic (Reverse)",
                () -> robotContainer.flyWheels.sysIdRoutines[0].dynamic(SysIdRoutine.Direction.kReverse)
        );
        testsChooser.addOption(
                "FlyWheel0 SysId Quasi Static (Forward)",
                () -> robotContainer.flyWheels.sysIdRoutines[0].quasistatic(SysIdRoutine.Direction.kForward)
        );
        testsChooser.addOption(
                "FlyWheel0 SysId Quasi Static (Reverse)",
                () -> robotContainer.flyWheels.sysIdRoutines[0].quasistatic(SysIdRoutine.Direction.kReverse)
        );
        return testsChooser;
    }
}
