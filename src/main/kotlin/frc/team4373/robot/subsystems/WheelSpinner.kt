package frc.team4373.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team4373.robot.RobotMap
import frc.team4373.robot.commands.wheelspinner.WheelSpinnerCommand

object WheelSpinner: Subsystem() {
    enum class Color {
        BLUE, GREEN, RED, YELLOW
    }

    @JvmStatic fun getInstance(): WheelSpinner = WheelSpinner

    private val wheelSpinnerMotor: WPI_TalonSRX
    private val deployServo: Servo
    private val colorSensor: ColorSensorV3

    init {
        wheelSpinnerMotor = WPI_TalonSRX(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.id)
        wheelSpinnerMotor.inverted = RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.inverted
        wheelSpinnerMotor.setNeutralMode(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.neutralMode)
        wheelSpinnerMotor.setSensorPhase(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.encoderPhase)
        wheelSpinnerMotor.config_kF(
            RobotMap.PID_IDX,
            RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kF
        )
        wheelSpinnerMotor.config_kP(
            RobotMap.PID_IDX,
            RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kP
        )
        wheelSpinnerMotor.config_kI(
            RobotMap.PID_IDX,
            RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kI
        )
        wheelSpinnerMotor.config_kD(
            RobotMap.PID_IDX,
            RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kD
        )
        deployServo = Servo(RobotMap.WHEEL_SPINNER_DEPLOY_SERVO_PORT)
        colorSensor = ColorSensorV3(RobotMap.COLOR_SENSOR_PORT)
    }

    /**
     * Spins the wheel spinner at the default percent output.
     */
    fun spin() {
        wheelSpinnerMotor.set(ControlMode.PercentOutput, RobotMap.SPINNER_SPEED)
    }

    /**
     * Stops the wheel spinner.
     */
    fun stopSpinner() {
        wheelSpinnerMotor.stopMotor()
    }

    /**
     * Returns the current percent output of the wheel spinner.
     * @return the current percent output, [-1, 1].
     */
    val wheelSpinnerPercentOutput: Double
        get() = wheelSpinnerMotor.motorOutputPercent

    /**
     * Deploys the spinner.
     */
    fun deploySpinner() {
        deployServo.set(RobotMap.SPINNER_SERVO_DEPLOY_ANGLE)
    }

    /**
     * Retracts the spinner.
     */
    fun retractSpinner() {
        deployServo.set(RobotMap.SPINNER_SERVO_RETRACT_ANGLE)
    }

    /**
     * Gets the color currently visible to the color sensor.
     * @return the currently visible [Color] value.
     */
    val color: Color
        get() {
            val red: Double = colorSensor.red.toDouble()
            val blue: Double = colorSensor.blue.toDouble()
            val green: Double = colorSensor.green.toDouble()
            val channelSum = red + blue + green
            return if (red / channelSum > RobotMap.RED_THRESHOLD) {
                Color.RED
            } else if (blue / channelSum > RobotMap.BLUE_THRESHOLD) {
                if (green / channelSum > RobotMap.GREEN_THRESHOLD) {
                    Color.GREEN
                } else {
                    Color.BLUE
                }
            } else {
                Color.YELLOW
            }
        }

    /**
     * Whether the servo is currently deployed.
     * @return true if the servo is retracted and false otherwise.
     */
    val isRetracted: Boolean
        get() = (Math.abs(deployServo.get() - RobotMap.SPINNER_SERVO_DEPLOY_ANGLE)
                < RobotMap.FP_EQUALITY_THRESHOLD)

    override fun initDefaultCommand() {
        defaultCommand = WheelSpinnerCommand()
    }
}
/*
package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.wheelspinner.WheelSpinnerCommand;

public class WheelSpinner extends Subsystem {
    public enum Color {
        BLUE, GREEN, RED, YELLOW
    }

    private static volatile WheelSpinner instance;

    /**
     * The getter for the WheelSpinner class.
     * @return the singleton WheelSpinner object.
     */
    public static WheelSpinner getInstance() {
        if (instance == null) {
            synchronized (WheelSpinner.class) {
                if (instance == null) {
                    instance = new WheelSpinner();
                }
            }
        }
        return instance;
    }

    private WPI_TalonSRX wheelSpinnerMotor;
    private Servo deployServo;
    private ColorSensorV3 colorSensor;

    private WheelSpinner() {
        this.wheelSpinnerMotor = new WPI_TalonSRX(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.id);
        this.wheelSpinnerMotor.setInverted(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.inverted);
        this.wheelSpinnerMotor.setNeutralMode(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.neutralMode);
        this.wheelSpinnerMotor.setSensorPhase(RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.encoderPhase);

        this.wheelSpinnerMotor.config_kF(RobotMap.PID_IDX,
                RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kF);
        this.wheelSpinnerMotor.config_kP(RobotMap.PID_IDX,
                RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kP);
        this.wheelSpinnerMotor.config_kI(RobotMap.PID_IDX,
                RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kI);
        this.wheelSpinnerMotor.config_kD(RobotMap.PID_IDX,
                RobotMap.WHEEL_SPINNER_MOTOR_CONFIG.gains.kD);

        this.deployServo = new Servo(RobotMap.WHEEL_SPINNER_DEPLOY_SERVO_PORT);
        this.colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);
    }

    /**
     * Spins the wheel spinner at the default percent output.
     */
    public void spin() {
        wheelSpinnerMotor.set(ControlMode.PercentOutput, RobotMap.SPINNER_SPEED);
    }

    /**
     * Stops the wheel spinner.
     */
    public void stopSpinner() {
        wheelSpinnerMotor.stopMotor();
    }

    /**
     * Returns the current percent output of the wheel spinner.
     * @return the current percent output, [-1, 1].
     */
    public double getWheelSpinnerPercentOutput() {
        return this.wheelSpinnerMotor.getMotorOutputPercent();
    }

    /**
     * Deploys the spinner.
     */
    public void deploySpinner() {
        deployServo.set(RobotMap.SPINNER_SERVO_DEPLOY_ANGLE);
    }

    /**
     * Retracts the spinner.
     */
    public void retractSpinner() {
        deployServo.set(RobotMap.SPINNER_SERVO_RETRACT_ANGLE);
    }

    /**
     * Gets the color currently visible to the color sensor.
     * @return the currently visible {@link Color} value.
     */
    public Color getColor() {
        double red = colorSensor.getRed();
        double blue = colorSensor.getBlue();
        double green = colorSensor.getGreen();
        double channelSum = red + blue + green;

        if (red / channelSum > RobotMap.RED_THRESHOLD) {
            return Color.RED;
        } else if (blue / channelSum > RobotMap.BLUE_THRESHOLD) {
            if (green / channelSum > RobotMap.GREEN_THRESHOLD) {
                return Color.GREEN;
            } else {
                return Color.BLUE;
            }
        } else {
            return Color.YELLOW;
        }
    }

    /**
     * Whether the servo is currently deployed.
     * @return true if the servo is retracted and false otherwise.
     */
    public boolean isRetracted() {
        return Math.abs(deployServo.get() - RobotMap.SPINNER_SERVO_DEPLOY_ANGLE)
                < RobotMap.FP_EQUALITY_THRESHOLD;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new WheelSpinnerCommand());
    }
}
 */