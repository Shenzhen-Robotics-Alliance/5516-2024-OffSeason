package frc.robot.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.utils.LEDAnimation;

public class LEDTest extends SequentialCommandGroup {
    public LEDTest(LEDStatusLight statusLight) {
        super.addCommands(statusLight.playAnimationAndStop(
                new LEDAnimation.SlideBackAndForth(0,200, 255, 1, 0.8),
                3
        ));

        super.addCommands(statusLight.playAnimationAndStop(
                new LEDAnimation.Rainbow(0.6),
                5
        ));
    }
}
