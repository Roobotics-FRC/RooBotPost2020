package frc.team4373.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.camera.VisionQuerierCommand;
import frc.team4373.robot.commands.drivetrain.DriveDistanceAuton;
import frc.team4373.robot.commands.drivetrain.RequirementFreeRotateAngleOffsetAuton;
import frc.team4373.robot.commands.intake.IntakeReleaseCommand;
import frc.team4373.robot.commands.shooter.ShooterShootCommand;

/**
 * An auton command that shoots and then gets in position to intake for teleop.
 *
 * <p>It does the following:
 * <ul>
 *     <li>Aligns to target.
 *     <li>Shoots balls.
 *     <li>Drives back 100 inches (to around the intake zone).
 * </ul>
 */
public class ShootThenPrepIntakeAuton extends SequentialCommandGroup {
    /**
     * Constructs a new shoot-then-drive-to-intake-zone auton.
     */
    public ShootThenPrepIntakeAuton() {
        addCommands(
                new VisionQuerierCommand(RobotMap.VISION_ANG_OFFSET_FIELD,
                        RobotMap.VISION_ALIGN_ALLOWABLE_OFFSET_DEG,
                        RequirementFreeRotateAngleOffsetAuton::new),
                new ShooterShootCommand(RobotMap.AUTON_LINE_SHOOT_SPEED)
                        .withTimeout(RobotMap.AUTON_SHOOT_TIME_SEC),
                new WaitCommand(RobotMap.SHOOTER_TIME_TO_SPIN_UP_SEC),
                new IntakeReleaseCommand(),
                new DriveDistanceAuton(100, 0.2, 90) // -90 is forward at start
        );
    }
}
