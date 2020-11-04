package frc.team4373.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.camera.VisionQuerierCommand;
import frc.team4373.robot.commands.drivetrain.RequirementFreeRotateAngleOffsetAuton;
import frc.team4373.robot.commands.drivetrain.TimedDriveAuton;
import frc.team4373.robot.commands.intake.IntakeReleaseCommand;
import frc.team4373.robot.commands.shooter.ShooterShootCommand;

/**
 * Drives off the initiation line, aims, and shoots.
 * <ol>
 *     <li>Drives backward from the initiation line.
 *     <li>Aims at the target.
 *     <li>Strafes right to line up the shooter instead of the camera.
 *     <li>Shoot for 5 seconds.
 * </ol>
 */
public class DriveThenShootAuton extends SequentialCommandGroup {

    /**
     * Drives off the initiation line, turns to the target, and shoots.
     *
     * <p>See the class documentation for details.
     */
    public DriveThenShootAuton() {
        addCommands(
                new TimedDriveAuton(1, 0.5, 90),
                new VisionQuerierCommand(RobotMap.VISION_ANG_OFFSET_FIELD,
                        RobotMap.VISION_ALIGN_ALLOWABLE_OFFSET_DEG,
                        RequirementFreeRotateAngleOffsetAuton::new),
                // FIXME: Change camera angle to include offset, then remove this line
                new TimedDriveAuton(0.55, 0.25, 0),
                new ShooterShootCommand(RobotMap.AUTON_LINE_SHOOT_SPEED)
                        .withTimeout(RobotMap.AUTON_SHOOT_TIME_SEC),
                new WaitCommand(RobotMap.SHOOTER_TIME_TO_SPIN_UP_SEC),
                new IntakeReleaseCommand()
        );
    }
}
