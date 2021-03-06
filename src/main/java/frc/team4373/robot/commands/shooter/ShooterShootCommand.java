package frc.team4373.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Shooter;

/**
 * Shoots the balls from the shooter.
 */
public class ShooterShootCommand extends CommandBase {
    protected Shooter shooter;

    private double velocity;
    private boolean vision;

    /**
     * Shoots the balls from the shooter at the specified velocity.
     * @param velocity the velocity at which to shoot the balls.
     */
    public ShooterShootCommand(double velocity) {
        addRequirements(this.shooter = Shooter.getInstance());
        this.velocity = velocity;
        this.vision = false;
    }

    /**
     * Shoots using the camera's distance computation to determine speed.
     */
    public ShooterShootCommand() {
        addRequirements(this.shooter = Shooter.getInstance());
        this.vision = true;
    }

    @Override
    public void initialize() {
        if (this.vision) {
            double distance = NetworkTableInstance.getDefault()
                    .getTable(RobotMap.VISION_TABLE_NAME).getEntry(RobotMap.VISION_DIST_FIELD)
                    .getDouble(-1);
            if (distance < 0) {
                this.velocity = 1;
                DriverStation.reportError("Illegal distance " + distance
                        + " read by shoot command; shooting at full speed", false);
            } else {
                this.velocity = percentVelocityForDistance(distance);
            }
        }
    }

    @Override
    public void execute() {
        double adjustedVelocity = velocity
                + OI.getInstance().getOperatorJoystick().getAxis(
                        RobotMap.OPER_ADJUST_SHOOT_SPEED_AXIS) / 10d;
        this.shooter.setVelocity(adjustedVelocity * RobotMap.SHOOTER_MAX_SPEED_NATIVE_UNITS);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stopShooter();
    }

    /**
     * Returns the velocity at which to shoot given a distance based on our model.
     * @param distance the distance from the target, in inches.
     * @return the percent of full velocity at which to shoot.
     */
    private double percentVelocityForDistance(double distance) {
        double raw = 1.06e-9 * Math.pow(distance, 4)
                - 8.68e-7 * Math.pow(distance, 3)
                + 2.59e-4 * Math.pow(distance, 2)
                - 0.0323 * distance
                + 2.13;
        return Utils.constrainPercentOutput(raw);
    }
}
