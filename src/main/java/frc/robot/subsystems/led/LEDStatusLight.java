package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.LEDAnimation;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends MapleSubsystem {
    private static final int DASHBOARD_DISPLAY_LENGTH = 16;
    private static AddressableLED led = null;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer bufferForDashboard;
    private final Timer t = new Timer();
    private LEDAnimation animation;

    private static final LEDAnimation DISABLED = new LEDAnimation.SlideBackAndForth(0,200, 255, 0.5, 0.8),
            ENABLED = new LEDAnimation.Rainbow(1);

    public LEDStatusLight(int port, int length) {
        super("LED");
        if (led != null) led.close();
        led = Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REAL ? new AddressableLED(port) : null;
        if (led != null) led.setLength(length);
        this.buffer = new AddressableLEDBuffer(length);
        this.bufferForDashboard = new AddressableLEDBuffer(DASHBOARD_DISPLAY_LENGTH);
        this.animation = DISABLED;
        t.start();
        if (led != null) led.start();

        super.setDefaultCommand(Commands.run(
                () -> setAnimation(DriverStation.isEnabled() ? ENABLED: DISABLED),
                this)
                .ignoringDisable(true));
    }

    final String[] colors = new String[DASHBOARD_DISPLAY_LENGTH/2];
    @Override
    public void periodic(double dt, boolean enabled) {
        animation.play(buffer, t.get());
        animation.play(bufferForDashboard, t.get());
        for (int i = 0; i < DASHBOARD_DISPLAY_LENGTH/2; i++)
            colors[i] = bufferForDashboard.getLED(DASHBOARD_DISPLAY_LENGTH/2 + i).toHexString();

        if (led != null) led.setData(buffer);
        if (Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REPLAY)
            Logger.recordOutput("Status Light", colors);
        else
            SmartDashboard.putStringArray("Status Light", colors);
    }

    public void setAnimation(LEDAnimation animation) {
        this.animation = animation;
    }

    public Command playAnimationAndStop(LEDAnimation animation, double durationSeconds) {
        return Commands.runOnce(() -> setAnimation(animation), this)
                .andThen(Commands.waitSeconds(durationSeconds))
                .andThen(Commands.runOnce(() -> setAnimation(
                        DriverStation.isEnabled() ? ENABLED : DISABLED
                )));
    }
}
