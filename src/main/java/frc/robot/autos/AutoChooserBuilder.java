package frc.robot.autos;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;

public class AutoChooserBuilder {
    public static LoggedDashboardChooser<Supplier<Auto>> buildAutoChooser(RobotContainer robotContainer) {
        final LoggedDashboardChooser<Supplier<Auto>> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("Amp Side 6 notes", () -> new AmpSideSixNotesFast(robotContainer));
        autoSendableChooser.addOption("None", Auto::none);

        return autoSendableChooser;
    }
}
