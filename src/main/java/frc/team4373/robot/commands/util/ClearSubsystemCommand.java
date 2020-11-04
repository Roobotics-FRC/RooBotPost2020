package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Temporarily takes control of a given subsystem so that it returns to its default command.
 */
public class ClearSubsystemCommand extends CommandBase {
    /**
     * Constructs a new ClearSubsystemCommand for a given subsystem.
     * @param subsystem the subsystem of which to take control.
     */
    public ClearSubsystemCommand(Subsystem subsystem) {
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}