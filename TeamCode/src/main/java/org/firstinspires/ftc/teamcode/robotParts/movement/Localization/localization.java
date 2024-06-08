package org.firstinspires.ftc.teamcode.robotParts.movement.Localization;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

public class localization {
    public DcMotorEx leftOdo,rightOdo,backOdo;
    final double[]
            L = {-2.6, 16.5},
            R = {-2.6,-8},
            B = {17, -2.5};
    final double
            WHEEL_RADIUS = 48,//in millimeters, this is for the odometry wheels
            GEAR_RATIO = 1/13.7,
            TICKS_PER_ROTATION = 8192,
            odoMultiplier = (72/38.6),
            ticksPerCM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS); //about 690 ticks per centimeter
    double oldX,oldY,oldTheta,currentX,currentY,currentTheta;
    //pos is in ticks, rest in cm or radian.
    public double oldLPos, oldRPos, oldBPos, currentLPos, currentRPos, currentBPos, dL, dR, dB, relDX, relDY, rSTR, rFWD, dForward, dTheta, dStrafe;
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

        currentLPos = leftOdo.getCurrentPosition();
        currentRPos = -rightOdo.getCurrentPosition();
        currentBPos = -backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * L[1] - dL * R[1])/(L[1] - R[1]); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(L[1] - R[1]);
        dStrafe = dB - B[0] * dTheta;

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

    /**
     * The Clueless linear odometry tracking equations.
     * For tuning, see {@link #arcLocalization()}
     */
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

        dForward = (dR * L[1] - dL * R[1])/(L[1] - R[1]); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(L[1] - R[1]);
        dStrafe = dB - B[0] * dTheta;

        relDX = dForward;
        relDY = dStrafe;

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }
}