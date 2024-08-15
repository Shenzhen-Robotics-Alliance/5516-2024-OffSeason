package frc.robot.autos;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;

public class AutoChooserBuilder {
    public static LoggedDashboardChooser<Supplier<Auto>> buildAutoChooser(RobotContainer robotContainer) {
        final LoggedDashboardChooser<Supplier<Auto>> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto::none);
        autoSendableChooser.addOption("Stage Side Eight Notes Prev", () -> new StageSideEightNotesPreview(robotContainer));

        return autoSendableChooser;
    }
}
