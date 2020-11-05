package frc.team4373.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.shooter.ShooterCommand

object Shooter: Subsystem() {
    @JvmStatic fun getInstance(): Shooter = Shooter

    private val shooterMotor1: WPI_TalonSRX
    private val shooterMotor2: WPI_TalonSRX

    init {
        val shooterMotor1Config = RobotMap.SHOOTER_MOTOR_1_CONFIG
        val shooterMotor2Config = RobotMap.SHOOTER_MOTOR_2_CONFIG
        shooterMotor1 = WPI_TalonSRX(shooterMotor1Config.id)
        shooterMotor2 = WPI_TalonSRX(shooterMotor2Config.id)
        shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder)
        shooterMotor1.inverted = shooterMotor1Config.inverted
        shooterMotor2.inverted = shooterMotor2Config.inverted
        shooterMotor1.setNeutralMode(shooterMotor1Config.neutralMode)
        shooterMotor2.setNeutralMode(shooterMotor2Config.neutralMode)
        shooterMotor1.setSensorPhase(shooterMotor1Config.encoderPhase)
        shooterMotor1.config_kF(RobotMap.PID_IDX, shooterMotor1Config.gains.kF)
        shooterMotor1.config_kP(RobotMap.PID_IDX, shooterMotor1Config.gains.kP)
        shooterMotor1.config_kI(RobotMap.PID_IDX, shooterMotor1Config.gains.kI)
        shooterMotor1.config_kD(RobotMap.PID_IDX, shooterMotor1Config.gains.kD)
        shooterMotor2.follow(shooterMotor1)
    }

    fun setPercentOutput(speed: Double) {
        shooterMotor1.set(ControlMode.PercentOutput, speed)
//        shooterMotor1[ControlMode.PercentOutput] = speed
    }

    val motor1PercentOutput: Double
        get() = shooterMotor1.motorOutputPercent
    val motor2PercentOutput: Double
        get() = shooterMotor2.motorOutputPercent

    /**
     * The velocity (in ENCODER UNITS)
     */
    var velocity: Double
        get() = shooterMotor1.selectedSensorVelocity.toDouble()
        set(speed) {
            shooterMotor1.set(ControlMode.Velocity, speed)
        }

    fun stopShooter() {
        shooterMotor1.stopMotor()
    }

    override fun initDefaultCommand() {
        defaultCommand = ShooterCommand()
    }

}