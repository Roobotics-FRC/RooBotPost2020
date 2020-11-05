package frc.team4373.robot.commands.drivetrain

import edu.wpi.first.wpilibj.command.Command
import frc.team4373.robot.RobotMap
import frc.team4373.robot.input.OI
import frc.team4373.robot.isZero
import frc.team4373.robot.subsystems.Drivetrain

class DrivetrainCommand : Command() {
    private var drivetrain: Drivetrain

    init {
        requires(Drivetrain.getInstance().also { drivetrain = it })
    }

    override fun execute() {
        var x = OI.getInstance().driveJoystick.rooGetX()
        var y = -OI.getInstance().driveJoystick.rooGetY()
        var rotation = OI.getInstance().driveJoystick.rooGetTwist()
        val slowMode = OI.getInstance().driveJoystick.getRawButton(
                RobotMap.DRIVE_SLOWER_SPEED_BUTTON)
        val brakeDisabled = OI.getInstance().driveJoystick.getRawButton(
                RobotMap.DRIVE_DISABLE_BRAKE_BUTTON)
        if (slowMode) {
            x /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
            y /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
            rotation /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
        }
        if (!isZero(x) || !isZero(y) || !isZero(rotation)) {
            drivetrain.drive(rotation, x, y)
        } else if (brakeDisabled || slowMode) {
            drivetrain.stop()
        } else {
            drivetrain.brake()
        }
    }

    override fun end() {
        drivetrain.stop()
    }

    override fun interrupted() {
        end()
    }

    override fun isFinished(): Boolean {
        return false
    }
}