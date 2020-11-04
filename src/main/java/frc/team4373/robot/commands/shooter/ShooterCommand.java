package frc.team4373.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
    private Shooter shooter;

    public ShooterCommand() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void execute() {
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
