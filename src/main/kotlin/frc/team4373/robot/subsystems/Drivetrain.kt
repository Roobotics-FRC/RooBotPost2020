package frc.team4373.robot.subsystems

import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.drivetrain.DriveStraightCommand
import frc.team4373.swerve.SwerveDrivetrain

/**
 * The swerve drivetrain subsystem.
 */
object Drivetrain: SwerveDrivetrain(RobotMap.getSwerveConfig()) {
    @JvmStatic fun getInstance(): Drivetrain = Drivetrain

    init {
        // Robot starts with shooter (right side) facing forward—compensate with 90° offset
        setPigeonYaw(-90.0)
    }

    override fun initDefaultCommand() {
        defaultCommand = DriveStraightCommand()
    }
}