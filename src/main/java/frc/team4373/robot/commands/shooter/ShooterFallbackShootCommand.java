package frc.team4373.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Shooter;

public class ShooterFallbackShootCommand extends CommandBase {
    private Shooter shooter;

    public ShooterFallbackShootCommand() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void execute() {
        double sliderVal = OI.getInstance().getDriveJoystick().rooGetThrottle();
        shooter.setVelocity(sliderVal * RobotMap.SHOOTER_MAX_SPEED_NATIVE_UNITS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
