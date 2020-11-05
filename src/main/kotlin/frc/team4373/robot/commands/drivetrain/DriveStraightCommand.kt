package frc.team4373.robot.commands.drivetrain

import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team4373.robot.RobotMap
import frc.team4373.robot.input.OI
import frc.team4373.robot.isZero
import frc.team4373.robot.subsystems.Drivetrain

class DriveStraightCommand : PIDCommand(RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kP, RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kI,
        RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kD) {
    private var lastManualRot: Long = 0
    private var drivetrain: Drivetrain

    /**
     * Constructs a new command that assists the driver in driving straight.
     */
    init {
        requires(Drivetrain.also { drivetrain = it })
    }
    override fun initialize() {
        this.setpoint = returnPIDInput()
    }

    override fun returnPIDInput(): Double {
        return drivetrain.angle
    }

    override fun usePIDOutput(rotPIDOutput: Double) {
        var x = OI.getInstance().driveJoystick.rooGetX()
        var y = -OI.getInstance().driveJoystick.rooGetY()
        var rotation = OI.getInstance().driveJoystick.rooGetTwist()
        val slowMode = OI.getInstance().driveJoystick.getRawButton(
                RobotMap.DRIVE_SLOWER_SPEED_BUTTON)
        if (slowMode) {
            x /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
            y /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
            rotation /= RobotMap.DRIVE_SLOWER_SPEED_FACTOR
        }
        val translating = !isZero(x) || !isZero(y)
        val rotating = !isZero(rotation)
        val cooledDown = (System.currentTimeMillis()
                > lastManualRot + RobotMap.DRIVE_STRAIGHT_COOLDOWN_MS)
        val brakeDisabled = OI.getInstance().driveJoystick.getRawButton(
                RobotMap.DRIVE_DISABLE_BRAKE_BUTTON)
        if (!rotating && !slowMode && translating && cooledDown) {
            // Not rotating or in slow mode & it's been long enough that we have a stable setpoint.
            // Drive straight.
            val rotAssist = rotPIDOutput * RobotMap.DRIVE_ASSIST_MAX_TURN_SPEED
            drivetrain.drive(rotAssist, x, y)
            return  // Break out to avoid setting a new setpoint—we want to maintain this one.
        } else if (rotating || translating) {
            // We haven't cooled down, we're moving in slow mode, or we're still rotating.
            // Drive normally.
            lastManualRot = System.currentTimeMillis()
            drivetrain.drive(rotation, x, y)
        } else if (brakeDisabled || slowMode) {
            // We're not moving, but the driver doesn't want to brake. Just stop.
            drivetrain.stop()
        } else {
            // We're not moving & we're okay to brake. Brake.
            drivetrain.brake()
        }
        this.setpoint = drivetrain.angle
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