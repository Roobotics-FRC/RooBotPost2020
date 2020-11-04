package frc.team4373.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Intake;

public class IntakeRetainCommand extends CommandBase {
    private Intake intake;

    public IntakeRetainCommand() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void execute() {
        this.intake.retainBall();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
