package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Drivetrain;

public class ResetNorthCommand extends CommandBase {
    private Drivetrain drivetrain;

    public ResetNorthCommand() {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        // this.drivetrain.resetPigeonYaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
