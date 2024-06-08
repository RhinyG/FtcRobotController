package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

//TODO: is extends RobotPart really worth it/the proper way?
public class MecanumDrivetrain extends RobotPart {
    public DcMotorEx FrontL, FrontR, BackL, BackR;
    final double[] motorWeights = {1.0,1.0,1.0,1.0};
    LinearOpMode myOpMode;

    final double
            angle = Math.PI * 1/3,
            xConstant = toCartesian(1,angle)[0],
            yConstant = toCartesian(1,angle)[1];
    double yL,yR,minYValue,powerMultiplier,maxPower;
    double[]
            motorPowers = {0,0,0,0},
            baseLVector = toPolar(xConstant, yConstant),
            baseRVector = toPolar(-xConstant, yConstant),
            LVector,RVector,sumVector;

    public MecanumDrivetrain(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
        myOpMode = opmode;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     */
    public void init(HardwareMap map, boolean TeleOp) {
        FrontL = map.get(DcMotorEx.class, "left_front");
        FrontR = map.get(DcMotorEx.class, "right_front");
        BackL = map.get(DcMotorEx.class, "left_back");
        BackR = map.get(DcMotorEx.class, "right_back");

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (TeleOp) {
            initIMU(map);
        }
    }

    public void unoptimizedDrive(double[] drivePower, double rotatePower) {
        LVector = new double[]{baseLVector[0],baseLVector[1]-drivePower[1]};
        RVector = new double[]{baseRVector[0],baseRVector[1]-drivePower[1]};

//        System.out.println("drivePower r "+drivePower[0]);
//        System.out.println("drivePower theta "+Math.toDegrees(drivePower[1]));
//        System.out.println("L rotate r "+LVector[0]);
//        System.out.println("L rotate theta "+Math.toDegrees(LVector[1]));
//        System.out.println("R rotate r "+RVector[0]);
//        System.out.println("R rotate theta "+Math.toDegrees(RVector[1]));

        if (Math.abs(LVector[1]) > 0.5 * Math.PI) {
            LVector[0] *= -1;
        }
        if (Math.abs(RVector[1]) > 0.5 * Math.PI) {
            RVector[0] *= -1;
        }

//        System.out.println("L negate r "+LVector[0]);
//        System.out.println("L negate theta "+Math.toDegrees(LVector[1]));
//        System.out.println("R negate r "+RVector[0]);
//        System.out.println("R negate theta "+Math.toDegrees(RVector[1]));

        yL = Math.abs(toCartesian(LVector)[1]);
        yR = Math.abs(toCartesian(RVector)[1]);
        minYValue = Math.min(yL, yR);
        LVector[0] *= minYValue/ yL;
        RVector[0] *= minYValue/ yR;

//        System.out.println("L y corrected r "+LVector[0]);
//        System.out.println("L y corrected theta "+Math.toDegrees(LVector[1]));
//        System.out.println("R y corrected r "+RVector[0]);
//        System.out.println("R y corrected theta "+Math.toDegrees(RVector[1]));
//        System.out.println("minL "+ yL);
//        System.out.println("minR "+ yR);
//        System.out.println("minY "+ minYValue);

        sumVector = toPolar(2 * toCartesian(LVector)[0] + 2 * toCartesian(RVector)[0],2 * toCartesian(LVector)[1] + 2 * toCartesian(RVector)[1]);
        powerMultiplier = 2*drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

//        System.out.println("checkVector r "+sumVector[0]);
//        System.out.println("checkVector theta "+Math.toDegrees(sumVector[1]+drivePower[1]));

        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

//        System.out.println("max "+maxPower);

        for (int i = 0; i < 4;i++) {
            motorPowers[i] /= (maxPower*drivePower[0]);
        }
//        System.out.println("FL "+motorPowers[0]);
//        System.out.println("FR "+motorPowers[1]);
//        System.out.println("BL "+motorPowers[2]);
//        System.out.println("BR "+motorPowers[3]);
        FrontL.setPower(motorPowers[0]);
        FrontR.setPower(motorPowers[1]);
        BackL.setPower(motorPowers[2]);
        BackR.setPower(motorPowers[3]);
        telemetry.addData("FL",motorPowers[0]);
        telemetry.addData("FR",motorPowers[1]);
        telemetry.addData("BL",motorPowers[2]);
        telemetry.addData("BR",motorPowers[3]);
    }
    public void driveBroken(double[] drivePower, double rotatePower) {
        LVector = new double[]{baseLVector[0],baseLVector[1]-drivePower[1]};
        RVector = new double[]{baseRVector[0],baseRVector[1]-drivePower[1]};

        telemetry.addData("drivePower r",drivePower[0]);
        telemetry.addData("drivePower theta",drivePower[1]);
        telemetry.addData("L rotate X",LVector[0]);
        telemetry.addData("L rotate Y",LVector[1]);
        telemetry.addData("R rotate X",RVector[0]);
        telemetry.addData("R rotate Y",RVector[1]);

        if (LVector[1] > 0.5 * Math.PI) {
            LVector[1] -= Math.PI;
        } else if (LVector[1] < -0.5 * Math.PI) {
            LVector[1] += Math.PI;
        }
        if (RVector[1] > 0.5 * Math.PI) {
            RVector[1] -= Math.PI;
        } else if (RVector[1] < -0.5 * Math.PI) {
            RVector[1] += Math.PI;
        }

        minYValue = Math.min(Math.abs(toCartesian(LVector)[1]),Math.abs(toCartesian(RVector)[1]));
        LVector[0] *= minYValue/LVector[1];
        RVector[0] *= minYValue/RVector[1];

        sumVector = toPolar(2 * toCartesian(LVector)[0] + 2 * toCartesian(RVector)[0],2 * toCartesian(LVector)[1] + 2 * toCartesian(RVector)[1]);
        powerMultiplier = drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        telemetry.addData("checkVector r",sumVector[0]);
        telemetry.addData("checkVector theta",sumVector[1]+drivePower[1]-0.5*Math.PI);

        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

        if (maxPower > drivePower[0]) {
            for (int i = 0; i < 4;i++) {
                motorPowers[i] /= (maxPower*drivePower[0]);
            }
        }
        FrontL.setPower(motorPowers[0]);
        FrontR.setPower(motorPowers[1]);
        BackL.setPower(motorPowers[2]);
        BackR.setPower(motorPowers[3]);
    }

    @Override
    public void runOpMode() {}
}