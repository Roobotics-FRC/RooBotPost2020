package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;


public class RotateAngleOffsetAuton extends ParallelRaceGroup {
    public RotateAngleOffsetAuton(double offset) {
        addCommands(new __RotateAngleOffsetAuton(offset),
                new WaitCommand(RobotMap.MAX_TURN_AUTON_TIME_SEC));
    }

    @SuppressWarnings("checkstyle:TypeName")
    private static class __RotateAngleOffsetAuton extends PIDCommand {
        private static final double MOTOR_OUTPUT_THRESHOLD = 0.2;
        private static final RobotMap.PID pid = new RobotMap.PID(0, 0.08, 0.05, 0.15);

        private Drivetrain drivetrain;
        private double offset;
        private double targetAngle;
        private boolean finished = false;

        public __RotateAngleOffsetAuton(double offset) {
            super(new PIDController(pid.kP, pid.kI, pid.kD),
                () -> 0,
                () -> 0,
                (output) -> { },
                Drivetrain.getInstance());
            this.drivetrain = Drivetrain.getInstance();
            this.offset = offset;
            this.m_measurement = this::_returnPIDInput;
            initialize();
            this.m_useOutput = this::_usePIDOutput;
        }

        @Override
        public boolean isFinished() {
            SmartDashboard.putNumber("targetAngle", targetAngle);
            return this.finished;
        }

        @Override
        public void initialize() {
            super.initialize();
            targetAngle = drivetrain.getPigeonYawRaw() + offset;
            this.setSetpoint(targetAngle);
            this.finished = false;
            // setTimeout(RobotMap.MAX_TURN_AUTON_TIME_SEC);
        }

        // private boolean _isFinished() {
        //     SmartDashboard.putNumber("targetAngle", targetAngle);
        //     return this.finished || this.isTimedOut();
        // }

        @SuppressWarnings("checkstyle:MethodName")
        private double _returnPIDInput() {
            return drivetrain.getPigeonYawRaw();
        }

        @SuppressWarnings("checkstyle:MethodName")
        private void _usePIDOutput(double output) {
            SmartDashboard.putNumber("output", output);
            if (Math.abs(output) <= MOTOR_OUTPUT_THRESHOLD) {
                this.finished = true;
                return;
            }
            drivetrain.drive(output * RobotMap.AUTON_TURN_SPEED, 0, 0);
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.stop();
        }

        private void setSetpoint(double setpoint) {
            this.m_setpoint = () -> setpoint;
        }
    }
}