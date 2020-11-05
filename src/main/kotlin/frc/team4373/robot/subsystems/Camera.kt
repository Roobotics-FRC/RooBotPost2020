package frc.team4373.robot.subsystems

import edu.wpi.first.wpilibj.command.Subsystem
import frc.team4373.robot.commands.camera.CameraCommand

/**
 * A symbolic subsystem that allows for tracking and killing commands that require only the camera.
 */
object Camera : Subsystem() {
    @JvmStatic fun getInstance(): Camera = Camera

    override fun initDefaultCommand() {
        defaultCommand = CameraCommand()
    }
}