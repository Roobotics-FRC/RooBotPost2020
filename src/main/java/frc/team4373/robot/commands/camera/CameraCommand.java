package frc.team4373.robot.commands.camera;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Camera;

/**
 * A placeholder command that maintains control of the camera subsystem.
 */
public class CameraCommand extends CommandBase {
    /**
     * Constructs a camera command.
     */
    public CameraCommand() {
        addRequirements(Camera.getInstance());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
