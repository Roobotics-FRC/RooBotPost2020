package frc.team4373.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.climber.ClimberCommand
import frc.team4373.robot.constrainPercentOutput
import java.lang.Double.max

object Climber : Subsystem() {
    @JvmStatic fun getInstance(): Climber = Climber

    private val lift: WPI_TalonSRX
    private val leftWinch: WPI_TalonSRX
    private val rightWinch: WPI_TalonSRX
    private val bottomLimitSwitch: DigitalInput
    private val topLimitSwitch: DigitalInput

    /**
     * Constructs a new Climber.
     */
    init {
        lift = WPI_TalonSRX(RobotMap.CLIMB_LIFT_CONFIG.id)
        leftWinch = WPI_TalonSRX(RobotMap.CLIMB_LEFT_WINCH_CONFIG.id)
        rightWinch = WPI_TalonSRX(RobotMap.CLIMB_RIGHT_WINCH_CONFIG.id)
        bottomLimitSwitch = DigitalInput(RobotMap.BOTTOM_LIMIT_SWITCH_DIO_PORT)
        topLimitSwitch = DigitalInput(RobotMap.TOP_LIMIT_SWITCH_DIO_PORT)

        lift.inverted = RobotMap.CLIMB_LIFT_CONFIG.inverted
        leftWinch.inverted = RobotMap.CLIMB_LEFT_WINCH_CONFIG.inverted
        rightWinch.inverted = RobotMap.CLIMB_RIGHT_WINCH_CONFIG.inverted

        lift.setNeutralMode(RobotMap.CLIMB_LIFT_CONFIG.neutralMode)
        leftWinch.setNeutralMode(RobotMap.CLIMB_LEFT_WINCH_CONFIG.neutralMode)
        rightWinch.setNeutralMode(RobotMap.CLIMB_RIGHT_WINCH_CONFIG.neutralMode)
    }

    /**
     * Sets the lift to extend (i.e., full power upward).
     */
    fun extendLift() {
        lift.set(ControlMode.PercentOutput, RobotMap.CLIMB_ELEVATOR_MOVE_SPEED)
    }

    /**
     * Sets the lift to retract (i.e., full power downward).
     */
    fun retractLift() {
        lift.set(ControlMode.PercentOutput, -RobotMap.CLIMB_ELEVATOR_MOVE_SPEED)
    }

    /**
     * Stops the lift (i.e., zero output).
     */
    fun stopLift() {
        lift.stopMotor()
    }

    /**
     * Raises winch 1 at the specified percent output (or stops it, if power = 0).
     * @param power the percent of full output at which to raise, [0, 1].
     */
    fun raiseLeftWinch(power: Double) {
        leftWinch.set(ControlMode.PercentOutput, constrainWinchOutput(power))
    }

    /**
     * Raises winch 2 at the specified percent output (or stops it, if power = 0).
     * @param power the percent of full output at which to raise, [0, 1].
     */
    fun raiseRightWinch(power: Double) {
        rightWinch.set(ControlMode.PercentOutput, constrainWinchOutput(power))
    }

    /**
     * Gets the state of the bottom limit switch.
     * @return true if the switch is activated, false otherwise.
     */
    fun getBottomLimitSwitch(): Boolean {
        return bottomLimitSwitch.get()
    }

    /**
     * Gets the state of the top limit switch.
     * @return true if the switch is activated, false otherwise.
     */
    fun getTopLimitSwitch(): Boolean {
        return topLimitSwitch.get()
    }

    /**
     * Safety-checks the power to be fed to a winch by clamping it to [-100%, 100%] of safe range
     * and then ensuring that it is positive—winches can only be raised, not lowered.
     * @param power the raw, unchecked percent-output value.
     * @return the power, constrained to safe bounds for the winch motors.
     */
    private fun constrainWinchOutput(power: Double): Double {
        return max(0.0, constrainPercentOutput(power)) * RobotMap.CLIMB_WINCH_MAX_SPEED
    }

    /**
     * Gets the percent output of the lift.
     * @return the percent output of the lift.
     */
    val liftPercentOutput: Double
        get() = lift.motorOutputPercent

    /**
     * Gets the percent output of the right winch motor.
     * @return the percent output of the right winch motor.
     */
    val rightWinchPercentOutput: Double
        get() = rightWinch.motorOutputPercent

    /**
     * Gets the percent output of the left winch motor.
     * @return the percent output of the left winch motor.
     */
    val leftWinchPercentOutput: Double
        get() = leftWinch.motorOutputPercent

    override fun initDefaultCommand() {
        defaultCommand = ClimberCommand()
    }
}