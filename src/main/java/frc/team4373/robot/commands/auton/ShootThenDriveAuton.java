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
 * Shoots and then drives forward off the initiation line.
 */
public class ShootThenDriveAuton extends SequentialCommandGroup {

    /**
     * Constructs a new shoot-then-drive auton.
     */
    public ShootThenDriveAuton() {
        addCommands(
                new VisionQuerierCommand(RobotMap.VISION_ANG_OFFSET_FIELD,
                        RobotMap.VISION_ALIGN_ALLOWABLE_OFFSET_DEG,
                        RequirementFreeRotateAngleOffsetAuton::new),
                new ShooterShootCommand(RobotMap.AUTON_LINE_SHOOT_SPEED)
                        .withTimeout(RobotMap.AUTON_SHOOT_TIME_SEC),
                new WaitCommand(RobotMap.SHOOTER_TIME_TO_SPIN_UP_SEC),
                new IntakeReleaseCommand(),
                new TimedDriveAuton(1, 0.5, 90)
        );
    }
}
