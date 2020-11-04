package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team4373.robot.RobotMap;

/**
 * A simple way to use a command with a PIDController.
 */
public abstract class RooPIDCommand extends CommandBase {
    private PIDController controller;

    public RooPIDCommand(double kP, double kI, double kD, Subsystem... requirements) {
        this.controller = new PIDController(kP, kI, kD);
        addRequirements(requirements);
    }

    public RooPIDCommand(RobotMap.PID pid, Subsystem... requirements) {
        this(pid.kP, pid.kI, pid.kD, requirements);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(getSetpoint());
        controller.reset();
    }

    @Override
    public void execute() {
        usePIDOutput(controller.calculate(returnPIDInput(), getSetpoint()));
    }

    public abstract boolean isFinished();

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double returnPIDInput();
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract void usePIDOutput(double output);
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double getSetpoint();

    protected PIDController getController() {
        return controller;
    }
}
