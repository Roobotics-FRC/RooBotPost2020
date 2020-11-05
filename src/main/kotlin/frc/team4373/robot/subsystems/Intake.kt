package frc.team4373.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.intake.IntakeCommand

object Intake: Subsystem() {
    @JvmStatic fun getInstance(): Intake = Intake

    private val groundIntakeMotor: WPI_TalonSRX
    private val uptakeIntakeMotor: WPI_TalonSRX
    private val servo: Servo

    init {
        groundIntakeMotor = WPI_TalonSRX(RobotMap.GROUND_INTAKE_MOTOR_CONFIG.id)
        groundIntakeMotor.inverted = RobotMap.GROUND_INTAKE_MOTOR_CONFIG.inverted
        groundIntakeMotor.setNeutralMode(RobotMap.GROUND_INTAKE_MOTOR_CONFIG.neutralMode)
        uptakeIntakeMotor = WPI_TalonSRX(RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.id)
        uptakeIntakeMotor.inverted = RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.inverted
        uptakeIntakeMotor.setNeutralMode(RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.neutralMode)
        servo = Servo(RobotMap.INTAKE_RELEASE_SERVO_PORT)
    }

    /**
     * Intakes a ball from the ground by running the ground and uptake motors.
     */
    fun intake() {
        groundIntakeMotor.set(ControlMode.PercentOutput, RobotMap.GROUND_INTAKE_SPEED)
        uptakeIntakeMotor.set(ControlMode.PercentOutput, RobotMap.UPTAKE_INTAKE_SPEED)
    }

    /**
     * Stops all (ground and uptake) intake motors.
     */
    fun stop() {
        groundIntakeMotor.stopMotor()
        uptakeIntakeMotor.stopMotor()
    }

    /**
     * Runs the ground intake motor in reverse to clear stuck balls. Also stops top intake.
     */
    fun reverseGroundIntake() {
        groundIntakeMotor.set(ControlMode.PercentOutput, -RobotMap.GROUND_INTAKE_SPEED)
        uptakeIntakeMotor.stopMotor()
    }

    /**
     * Sets the servo to the ball-release angle.
     */
    fun releaseBall() {
        servo.set(RobotMap.INTAKE_SERVO_RELEASE_ANGLE)
    }

    /**
     * Sets the servo to the ball-retention angle.
     */
    fun retainBall() {
        servo.set(RobotMap.INTAKE_SERVO_RETAIN_ANGLE)
    }

    /**
     * Gets the percent output of the ground intake motor.
     * @return the percent output of the ground intake motor.
     */
    val groundMotorPercentOutput: Double
        get() = groundIntakeMotor.motorOutputPercent

    /**
     * Gets the percent output of the uptake intake motor.
     * @return the percent output of the uptake intake motor.
     */
    val uptakeMotorPercentOutput: Double
        get() = uptakeIntakeMotor.motorOutputPercent

    /**
     * Returns whether the balls are currently being retained.
     * @return whether the servo is in the retain state.
     */
    val ballsAreRetained: Boolean
        get() = servo.get() == RobotMap.INTAKE_SERVO_RETAIN_ANGLE

    override fun initDefaultCommand() {
        defaultCommand = IntakeCommand()
    }
}