package frc.robot.autos;

import frc.robot.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooserBuilder {
    public static LoggedDashboardChooser<Auto> buildAutoChooser(RobotContainer robotContainer) {
        final LoggedDashboardChooser<Auto> autoSendableChooser = new LoggedDashboardChooser<>("Select Auto");
        autoSendableChooser.addDefaultOption("None", Auto.none());
        autoSendableChooser.addOption("Stage Side Eight Notes Prev", new StageSideEightNotesPreview(robotContainer));

        return autoSendableChooser;
    }
}
