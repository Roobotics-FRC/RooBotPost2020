package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4373.robot.RobotMap;

public class Intake extends SubsystemBase {
    private static volatile Intake instance;

    /**
     * The getter for the Intake class.
     * @return the singleton Intake object.
     */
    public static Intake getInstance() {
        if (instance == null) {
            synchronized (Intake.class) {
                if (instance == null) {
                    instance = new Intake();
                }
            }
        }
        return instance;
    }

    private WPI_TalonSRX groundIntakeMotor;
    private WPI_TalonSRX uptakeIntakeMotor;
    private Servo servo;

    private Intake() {
        this.groundIntakeMotor = new WPI_TalonSRX(RobotMap.GROUND_INTAKE_MOTOR_CONFIG.id);
        this.groundIntakeMotor.setInverted(RobotMap.GROUND_INTAKE_MOTOR_CONFIG.inverted);
        this.groundIntakeMotor.setNeutralMode(RobotMap.GROUND_INTAKE_MOTOR_CONFIG.neutralMode);

        this.uptakeIntakeMotor = new WPI_TalonSRX(RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.id);
        this.uptakeIntakeMotor.setInverted(RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.inverted);
        this.uptakeIntakeMotor.setNeutralMode(RobotMap.UPTAKE_INTAKE_MOTOR_CONFIG.neutralMode);

        this.servo = new Servo(RobotMap.INTAKE_RELEASE_SERVO_PORT);
    }

    /**
     * Intakes a ball from the ground by running the ground and uptake motors.
     */
    public void intake() {
        groundIntakeMotor.set(ControlMode.PercentOutput, RobotMap.GROUND_INTAKE_SPEED);
        uptakeIntakeMotor.set(ControlMode.PercentOutput, RobotMap.UPTAKE_INTAKE_SPEED);
    }

    /**
     * Stops all (ground and uptake) intake motors.
     */
    public void stop() {
        this.groundIntakeMotor.stopMotor();
        this.uptakeIntakeMotor.stopMotor();
    }

    /**
     * Runs the ground intake motor in reverse to clear stuck balls. Also stops top intake.
     */
    public void reverseGroundIntake() {
        this.groundIntakeMotor.set(ControlMode.PercentOutput, -RobotMap.GROUND_INTAKE_SPEED);
        this.uptakeIntakeMotor.stopMotor();
    }

    /**
     * Sets the servo to the ball-release angle.
     */
    public void releaseBall() {
        servo.set(RobotMap.INTAKE_SERVO_RELEASE_ANGLE);
    }

    /**
     * Sets the servo to the ball-retention angle.
     */
    public void retainBall() {
        servo.set(RobotMap.INTAKE_SERVO_RETAIN_ANGLE);
    }

    /**
     * Gets the percent output of the ground intake motor.
     * @return the percent output of the ground intake motor.
     */
    public double getGroundMotorPercentOutput() {
        return groundIntakeMotor.getMotorOutputPercent();
    }

    /**
     * Gets the percent output of the uptake intake motor.
     * @return the percent output of the uptake intake motor.
     */
    public double getUptakeMotorPercentOutput() {
        return uptakeIntakeMotor.getMotorOutputPercent();
    }

    /**
     * Returns whether the balls are currently being retained.
     * @return whether the servo is in the retain state.
     */
    public boolean getBallsAreRetained() {
        return servo.get() == RobotMap.INTAKE_SERVO_RETAIN_ANGLE;
    }

    @Override
    public void periodic() {

    }
}
