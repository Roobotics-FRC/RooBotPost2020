package frc.team4373.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    private static final double MIN_SERVO_RELEASE_TIME_SEC = 0.5;
    private Intake intake;
    private double depressTime;

    public IntakeCommand() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void execute() {
        // Ball intake/dislodgment from ground
        if (OI.getInstance().getOperatorJoystick().getRawButton(RobotMap.OPER_INTAKE_BUTTON)) {
            this.intake.intake();
        } else if (OI.getInstance().getOperatorJoystick().getRawButton(
                RobotMap.OPER_REVERSE_INTAKE_BUTTON)) {
            this.intake.reverseGroundIntake();
        } else {
            this.intake.stop();
        }

        // Ball release to shooter
        double now = Timer.getFPGATimestamp();
        if (OI.getInstance().getOperatorJoystick().getRawButtonPressed(
                RobotMap.OPER_BALL_RELEASE_BUTTON)) {
            this.depressTime = now;
        }

        if (now < depressTime + MIN_SERVO_RELEASE_TIME_SEC
                || OI.getInstance().getOperatorJoystick().getRawButton(
                        RobotMap.OPER_BALL_RELEASE_BUTTON)) {
            this.intake.releaseBall();
        } else {
            this.intake.retainBall();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
