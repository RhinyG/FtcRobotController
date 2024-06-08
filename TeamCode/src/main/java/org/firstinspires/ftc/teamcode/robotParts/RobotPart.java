package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

//TODO: rename
public abstract class RobotPart extends LinearOpMode {
    protected Map<String, DcMotorEx> motors = new HashMap<>();
    protected Map<String, Servo> servos = new HashMap<>();
    protected Map<String, CRServo> crServos = new HashMap<>();
    public IMU imu;
    public DcMotorEx leftOdo,rightOdo,backOdo;
    final double Lx = -7.2,Ly = 12.1,
            Rx = -7.2,Ry = -12.1,
            Bx = -16,By = 0,
            WHEEL_RADIUS = 48,//in millimeters, this is for the odometry wheels
            GEAR_RATIO = 1/13.7,
            TICKS_PER_ROTATION = 8192,
            odoMultiplier = (72/38.6),
            ticksPerCM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS); //about 690 ticks per centimeter
    double oldX,oldY,oldTheta,currentX,currentY,currentTheta;
    //pos is in ticks, rest in cm or radian.
    public double oldLPos, oldRPos, oldBPos, currentLPos, currentRPos, currentBPos, dL, dR, dB, relDX, relDY, rSTR, rFWD, dForward, dTheta, dStrafe;


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
    public double[] toPolar(double[] cartesian, boolean normalise) {
        return toPolar(cartesian[0],cartesian[1],normalise);
    }
    //TODO: documentation, EN
    public double[] toPolar(double[] cartesian) {
        return toPolar(cartesian[0],cartesian[1],false);
    }
    //TODO: documentation, EN
    public double[] toPolar(double x, double y) {
        return toPolar(x, y, false);
    }
    //TODO: documentation, EN
    public double[] toPolar(double x, double y, boolean normalise) {
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y,x);
        if (normalise) {
            if (theta > Math.PI) {
                theta -= 2 * Math.PI;
            } else if (theta <= -Math.PI) {
                theta += 2 * Math.PI;
            }
        }
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
    //TODO: documentation, EN
    //The Clueless linear odometry tracking equations
    public double[] linearLocalization() {
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;

        currentLPos = -leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * Ly - dL * Ry)/(Ly - Ry); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(Ly - Ry);
        dStrafe = dB - Bx * dTheta;

        relDX = dForward;
        relDY = dStrafe;

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }
    //TODO: documentation, EN
    /**
     * <i>Based on The Clueless constant velocity arc localization tutorial.</i>
     * <p>This is the Constant Velocity Arc Localization algorithm STT uses. It uses three odometry wheels and no IMU.</p>
     * <h2>Tuning</h2>
     * <p>It is very important to spend enough time on your tuning. If the earlier steps are not accurate, the later steps also won't be.</p><p></p>
     * <h3>Step 0</h3>
     * <p>Get a caliper and add values for Lx, Ly, Rx, Ry, Bx, By. Also add values for your gear ratio, ticks per rotation and (odometry) wheel radius.
     * This values should be known for the hardware you use.</p><p></p>
     * <h3>Step 1</h3>
     * <p>Check if the odometer directions are good. If you move the wheels manually, do they increase like they should, or do they decrease?
     * If so, you need to add or remove a negative before the Odo.getCurrentPosition.</p><p></p>
     * <h3>Step 2</h3>
     * <p>If step 1 has been tuned correctly, you can now drag the robot forward and get a semi-reasonable X value.
     * The next step is to roll the robot forward, preferably more than several meters, and check with a tape measure how close the variable is to reality.
     * To tune this, you need to change the odoMultiplier variable. You really want to get this to less than 1% error.</p><p></p>
     * <h3>Step 2</h3>
     * <p>The next step is to get the real value for your track width (Ly - Ry). One method is to rotate the robot 10 or 20 times,
     * and compare how much your theta is versus how much it should be. If the number is too small, your track width needs to decrease.</p><p></p>
     * <h3>Step 3</h3>
     * <p>The next step is to get the real value for your Bx. Rotate the robot a full rotation, which should mean your back encoder is in the same spot as it started.
     * If your Y value does not return to zero, Bx needs to be changed. If Y is too high, increase/decrease Bx.</p><p></p>
     * @return An array containing the global X and Y coordinates and heading of the robot on the field.
     */
    public double[] arcLocalization() {
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;

        currentLPos = -leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * Ly - dL * Ry)/(Ly - Ry); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(Ly - Ry);
        dStrafe = dB - Bx * dTheta;

        //If dTheta == 0, revert to linearLocalization because then you have a circle with infinite diameter, which it can't (because it divides by zero). Luckily a circle with infinite diameter is just a line.
        if (dTheta == 0) {
            relDX = dForward;
            relDY = dStrafe;
        } else {
            rFWD = dForward / dTheta;
            rSTR = dStrafe / dTheta;

            relDX = rFWD * Math.sin(dTheta) - rSTR * (1 - Math.cos(dTheta));
            relDY = rSTR * Math.sin(dTheta) + rFWD * (1 - Math.cos(dTheta));
        }

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }
    //TODO: getCurrentHeadingRadians
    //TODO: getCurrentHeadingDegrees
    //TODO: calibrateEncoders
    //TODO: stop
    //TODO: checkDirection
}