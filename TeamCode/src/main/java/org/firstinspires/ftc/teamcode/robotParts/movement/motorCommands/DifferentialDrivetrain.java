package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

@Config
public class DifferentialDrivetrain extends RobotPart {
    public static int rotations = 0;
    double x, y, r, gamepadTheta, pidR, pidB, TICKS_PER_ROTATION = 537.7/15*26;
    int redCurrentPos, blueCurrentPos, FBpos, FRpos,BBpos,BRpos, redHeadingGoal = 0, blueHeadingGoal = 0;
    public DcMotorEx FrontB,FrontR,BackB,BackR;
    public DcMotorEx[] DTMotors;
    String[] DTMotorNames = {"blue_front","red_front","blue_back","red_back"};
    int[] encoderPositions = {FBpos,FRpos,BBpos,BRpos};
    public static double pb = 0.0025, ib = 0.001, db = 0.00004, pr = 0.0025, ir = 0.001, dr = 0.00004;
    PIDController blueHeading = new PIDController(pb, ib, db), redHeading = new PIDController(pr,ir,dr);
    LinearOpMode myOpMode;

    public DifferentialDrivetrain(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
        myOpMode = opmode;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * This methods initialises the swerve drivetrain and the IMU and sets all the directions and modes to their correct settings.
     */
    //TODO: split into initAutonomous en initTeleOp
    public void initRobot(HardwareMap hardwareMap) {
        DTMotors = new DcMotorEx[]{FrontB, FrontR, BackB, BackR};
        for (int i = 0; i < DTMotors.length; i++) {
            DTMotors[i] = hardwareMap.get(DcMotorEx.class,DTMotorNames[i]);
            if (i < 2){
                DTMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                DTMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            DTMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            DTMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        initIMU(hardwareMap);
    }
    //TODO: documentation
    //TODO: in EN, then delete
    public void simple(){
        double FWDB = myOpMode.gamepad1.left_stick_y;
        double ROTB = -0.55 * myOpMode.gamepad1.left_stick_x;
        double FWDR = myOpMode.gamepad1.right_stick_y;
        double ROTR = -0.55 * myOpMode.gamepad1.right_stick_y;
        DTMotors[0].setPower(FWDB + ROTB);
        DTMotors[1].setPower(FWDR + ROTR);
        DTMotors[2].setPower(FWDB - ROTB);
        DTMotors[3].setPower(FWDR - ROTR);
    }
    //TODO: documentation
    public void lessSimple() {
        x = myOpMode.gamepad1.left_stick_x;
        y = -myOpMode.gamepad1.left_stick_y;
        r = Math.sqrt(x * x + y * y);
        int Bpos = DTMotors[3].getCurrentPosition(),Fpos = -DTMotors[1].getCurrentPosition();
        redCurrentPos = Fpos - Bpos;
        DTMotors[1].setPower(myOpMode.gamepad1.left_stick_y);
        DTMotors[3].setPower(myOpMode.gamepad1.right_stick_y);
    }
    //TODO: documentation
    public void singleJoyStickPID() {
        x = myOpMode.gamepad1.left_stick_x;
        y = -myOpMode.gamepad1.left_stick_y;
        r = toPolar(x,y)[0];
        gamepadTheta = toPolar(x,y)[1];

        for (int i = 0; i < DTMotors.length; i++) {
            encoderPositions[i] = DTMotors[i].getCurrentPosition();
        }

        redCurrentPos = encoderPositions[1] - encoderPositions[3];
        blueCurrentPos = encoderPositions[0] - encoderPositions[2];

        if (myOpMode.gamepad1.x) {
            for (DcMotorEx motor : DTMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }


        if(r > 0.3){
            //TODO: stop rotation all the way back, maybe with gm0/FTClib button press to event?
            redHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION);
//            if ((redHeadingGoal - redCurrentPos) < TICKS_PER_ROTATION) {
//                rotations++;
//            }
//            if (redHeadingGoal - redCurrentPos > -TICKS_PER_ROTATION) {
//                rotations--;
//            }
            redHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION + rotations * 2 * TICKS_PER_ROTATION);
            blueHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION + rotations * 2 * TICKS_PER_ROTATION);
        }

        redHeading.setPID(pr,ir,dr);
        blueHeading.setPID(pb,ib,db);
        pidR = redHeading.calculate(redCurrentPos,redHeadingGoal);
        pidB = blueHeading.calculate(blueCurrentPos,blueHeadingGoal);

        DTMotors[0].setPower(pidB - r);
        DTMotors[1].setPower(pidR + r);
        DTMotors[2].setPower(-pidB - r);
        DTMotors[3].setPower(-pidR + r);
        telemetry.addData("goal",redHeadingGoal);
        telemetry.addData("pos",redCurrentPos);
        telemetry.addData("diff",redHeadingGoal - redCurrentPos);
        telemetry.addData("rots",rotations);
    }
    public void doubleJoyStickPID() {
        double xR = myOpMode.gamepad1.right_stick_x;
        double xB = myOpMode.gamepad1.left_stick_x;
        double yR = -myOpMode.gamepad1.right_stick_y;
        double yB = -myOpMode.gamepad1.left_stick_y;
        double rR = toPolar(xR,yR)[0];
        double rB = toPolar(xB,yB)[0];
        double thetaB = toPolar(xB,yB)[1];
        double thetaR = toPolar(xR,yR)[1];

        for (int i = 0; i < DTMotors.length; i++) {
            encoderPositions[i] = DTMotors[i].getCurrentPosition();
        }

        redCurrentPos = encoderPositions[1] - encoderPositions[3];
        blueCurrentPos = encoderPositions[0] - encoderPositions[2];

        if (myOpMode.gamepad1.x) {
            for (DcMotorEx motor : DTMotors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if(rB > 0.5){
            blueHeadingGoal = (int) ((thetaB-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION);
        }

        if(rR > 0.5){
            redHeadingGoal = (int) ((thetaR-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION);
        }

        redHeading.setPID(pr,ir,dr);
        blueHeading.setPID(pb,ib,db);
        pidR = redHeading.calculate(redCurrentPos,redHeadingGoal);
        pidB = blueHeading.calculate(blueCurrentPos,blueHeadingGoal);

        DTMotors[0].setPower(pidB - r);
        DTMotors[1].setPower(pidR + r);
        DTMotors[2].setPower(-pidB - r);
        DTMotors[3].setPower(-pidR + r);
    }

    //TODO: motor power algorithm based off auton

    public void runOpMode() throws InterruptedException {

    }
}