package frc.team4373.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.subsystems.Intake;

public class IntakeReleaseCommand extends CommandBase {
    private static double WAIT_ITERATIONS = 25;
    private Intake intake;
    private double iterations = 0;

    public IntakeReleaseCommand() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void initialize() {
        this.iterations = 0;
    }

    @Override
    public void execute() {
        if (++this.iterations >= WAIT_ITERATIONS) {
            this.intake.releaseBall();
        }
    }

    @Override
    public boolean isFinished() {
        return this.iterations > WAIT_ITERATIONS;
    }
}
