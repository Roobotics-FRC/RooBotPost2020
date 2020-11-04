package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Drivetrain;

import static frc.team4373.robot.Utils.isZero;

public class DrivetrainCommand extends CommandBase {
    private final Drivetrain drivetrain;

    public DrivetrainCommand() {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double x = OI.getInstance().getDriveJoystick().rooGetX();
        double y = -OI.getInstance().getDriveJoystick().rooGetY();
        double rotation = OI.getInstance().getDriveJoystick().rooGetTwist();

        boolean slowMode = OI.getInstance().getDriveJoystick().getRawButton(
                RobotMap.DRIVE_SLOWER_SPEED_BUTTON);
        boolean brakeDisabled = OI.getInstance().getDriveJoystick().getRawButton(
                RobotMap.DRIVE_DISABLE_BRAKE_BUTTON);

        if (slowMode) {
            x /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR;
            y /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR;
            rotation /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR;
        }

        if (!isZero(x) || !isZero(y) || !isZero(rotation)) {
            this.drivetrain.drive(rotation, x, y);
        } else if (brakeDisabled || slowMode) {
            this.drivetrain.stop();
        } else {
            this.drivetrain.brake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
