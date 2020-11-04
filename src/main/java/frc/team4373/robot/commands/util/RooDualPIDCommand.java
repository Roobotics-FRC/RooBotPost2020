package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team4373.robot.RobotMap;

/**
 * A simple way to use a command with two PIDControllers.
 */
public abstract class RooDualPIDCommand extends CommandBase {
    private PIDController controller1;
    private PIDController controller2;

    /**
     * Creates a dual PID command with the specified gains.
     * @param kP1 the proportional gain for the first PID controller.
     * @param kI1 the integral gain for the first PID controller.
     * @param kD1 the derivative gain for the first PID controller.
     * @param kP2 the proportional gain for the second PID controller.
     * @param kI2 the integral gain for the second PID controller.
     * @param kD2 the derivative gain for the second PID controller.
     * @param requirements the required subsystems.
     */
    public RooDualPIDCommand(double kP1, double kI1, double kD1,
                             double kP2, double kI2, double kD2, Subsystem... requirements) {
        this.controller1 = new PIDController(kP1, kI1, kD1);
        this.controller2 = new PIDController(kP2, kI2, kD2);
        addRequirements(requirements);
    }

    /**
     * Creates a dual PID command with the specified gains.
     * @param pid1 the gains for the first pID controller.
     * @param pid2 the gains for the second PID controller.
     * @param requirements the required subsystems.
     */
    public RooDualPIDCommand(RobotMap.PID pid1, RobotMap.PID pid2, Subsystem... requirements) {
        this(pid1.kP, pid1.kI, pid1.kD, pid2.kP, pid2.kI, pid2.kD, requirements);
    }

    @Override
    public void initialize() {
        controller1.setSetpoint(getSetpoint1());
        controller1.reset();
        controller2.setSetpoint(getSetpoint2());
        controller2.reset();
    }

    @Override
    public void execute() {
        usePIDOutput(
            controller1.calculate(returnPIDInput1(), getSetpoint1()),
            controller2.calculate(returnPIDInput2(), getSetpoint2())
        );
    }

    public abstract boolean isFinished();

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double returnPIDInput1();
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double returnPIDInput2();
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract void usePIDOutput(double output1, double output2);
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double getSetpoint1();
    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected abstract double getSetpoint2();

    @SuppressWarnings("checkstyle:EmptyLineSeparator")
    protected PIDController getController1() {
        return controller1;
    }
    protected PIDController getController2() {
        return controller2;
    }
}