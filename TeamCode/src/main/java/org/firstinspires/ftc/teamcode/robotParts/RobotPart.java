package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

//TODO: rename
public abstract class RobotPart extends LinearOpMode {//TODO: extends OpMode, like PIDF_test
    protected Map<String, DcMotorEx> motors = new HashMap<>();
    protected Map<String, Servo> servos = new HashMap<>();
    protected Map<String, CRServo> crServos = new HashMap<>();
    IMU imu;

    public void resetEncoders() {
        for (DcMotorEx motor : motors.values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setAllPowers(double power) {
        for (DcMotorEx motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void allCrPower(double power) {
        for (CRServo servo : crServos.values()) {
            servo.setPower(power);
        }
    }
    //TODO: documentation
    public void initIMU(HardwareMap map){
        imu = map.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }
    /**
     * ResetYaw is one of the new IMU methods, and it resets the yaw of the robot. When implemented correctly,
     * yaw is the only rotate axis you want to change.
     */
    public void resetYaw() {
        imu.resetYaw();
    }
    //TODO: documentation, EN
    public double[] oldToPolar(double[] cartesian) {
        return oldToPolar(cartesian[0],cartesian[1]);
    }
    //TODO: documentation, EN
    public double[] oldToPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double theta;
        if (x >= 0 && y >= 0) {
            theta = Math.atan(y / x);
        } else if (x<0) {
            theta = Math.atan(y / x) + Math.PI;
        } else {
            theta = Math.atan(y / x) + 2 * Math.PI;
        }
        return new double[]{r,theta};
    }
    //TODO: documentation, EN
    public double[] toPolar(double[] cartesian) {
        return toPolar(cartesian[0],cartesian[1]);
    }
    //TODO: documentation, EN
    public double[] toPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y,x);
        return new double[]{r,theta};
    }
    //TODO: documentation, EN
    public double[] toCartesian(double[] polar) {
        return toCartesian(polar[0],polar[1]);
    }
    //TODO: documentation, EN
    public double[] toCartesian(double r, double theta) {
        double x = r * Math.cos(theta);
        double y = r * Math.sin(theta);
        return new double[]{x,y};
    }
    //TODO: getCurrentHeadingRadians
    //TODO: getCurrentHeadingDegrees
    //TODO: calibrateEncoders
    //TODO: stop
    //TODO: checkDirection
    //TODO: new tracking algorithm
    //TODO: new pathfinding algorithm
}
