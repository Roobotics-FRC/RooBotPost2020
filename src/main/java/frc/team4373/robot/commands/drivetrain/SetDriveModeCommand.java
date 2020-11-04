package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Drivetrain;

public class SetDriveModeCommand extends CommandBase {
    private Drivetrain.DriveMode mode;
    private Drivetrain drivetrain;

    public SetDriveModeCommand(Drivetrain.DriveMode mode) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
        this.mode = mode;
    }

    @Override
    public void execute() {
        this.drivetrain.setDriveMode(this.mode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
