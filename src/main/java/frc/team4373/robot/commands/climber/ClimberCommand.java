package frc.team4373.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Climber;

public class ClimberCommand extends CommandBase {
    private Climber climber;

    public ClimberCommand() {
        addRequirements(this.climber = Climber.getInstance());
    }

    @Override
    public void execute() {
        switch (OI.getInstance().getOperatorJoystick().getPOV()) {
            case 0:
            case 45:
            case 315:
                if (!climber.getTopLimitSwitch()) {
                    climber.extendLift();
                } else {
                    climber.stopLift();
                }
                break;
            case 135:
            case 180:
            case 225:
                if (!climber.getBottomLimitSwitch()) {
                    climber.retractLift();
                } else {
                    climber.stopLift();
                }
                break;
            default:
                climber.stopLift();
                break;
        }
        this.climber.raiseLeftWinch(OI.getInstance().getOperatorJoystick()
                .getAxis(RobotMap.OPER_RAISE_L_WINCH_AXIS));
        this.climber.raiseRightWinch(OI.getInstance().getOperatorJoystick()
                .getAxis(RobotMap.OPER_RAISE_R_WINCH_AXIS));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.climber.stopLift();
        this.climber.raiseLeftWinch(0);
        this.climber.raiseRightWinch(0);
    }
}
