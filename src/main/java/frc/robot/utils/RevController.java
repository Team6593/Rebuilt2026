package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of the CommandXboxController, but for the rev controller.
 */
public class RevController extends CommandXboxController{
    private final CommandXboxController m_hid;

    /**
     * Construct an instance of a controller.
     * 
     * @param port - The port index on the DS that the controller is plugged into.
     */
    public RevController(int port) {
        super(port);
        m_hid = new CommandXboxController(port);
    }

    /**
     * Get the underlying CommandXboxControlelr object.
     * 
     * @return the wrapped CommandXboxController object,
     */
    public CommandXboxController getCommandXboxController() {
        return m_hid;
    }

    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    
}
