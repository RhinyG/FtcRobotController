////TODO: change name of file to something better
////TODO: see if RobotCentric and FieldCentric work here as well
//package org.firstinspires.ftc.teamcode.robotParts.movement.drive;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
////TODO: rename to MecanumDrivetrain or make that work like this
//public class newAutonMethods {
//    //TODO: explain variables
//    private final LinearOpMode myOpMode;
//    private final ElapsedTime runtime = new ElapsedTime();
//    Telemetry telemetry;
//    HardwareMap map;
//    //TODO: see what happens if you change this to DcMotorEx
//    public DcMotor FrontL,FrontR,BackL,BackR;
//    public IMU imu;
//
//
//    /**
//     * This methods initialises the mecanum drivetrain and the IMU and sets all the directions and modes to their correct settings.
//     */
//    public void initRobot() {
//        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
//
//        //TODO: see what happens if you change this to DcMotorEx
//        FrontL = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
//        FrontR = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
//        BackL = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
//        BackR = myOpMode.hardwareMap.get(DcMotor.class, "right_back");
//
//        //TODO: see what happens if you change this to DcMotorEx
//        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
//        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackR.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        resetYaw();
//        telemetry.addData("heading",getCurrentHeading());
//        telemetry.update();
//    }
//
//    /**
//     * This method is a shorthand for the regular linearDrive, with the speed, telemetry and stopTime filled in already.
//     * This makes long strings of auton with similar driveY's easier to write and read.
//     * @param x - How far you want to strafe, in centimeters
//     * @param y - How far you want to drive forward, in centimeters
//     */
//    public void linearDrive(double x, double y) {linearDrive(x, y, 0.7, 4000);}
//    //TODO: documentation
//    public void linearDrive(double x, double y, double max_speed, double stopTime) {
//        calibrateEncoders();
//        beginTime = System.currentTimeMillis();
//        double turn;
//        heading = current_target_heading;
//        tickY = (int) (y * ourTicksPerCM);
//        tickX = (int) (x * ourTicksPerCM);
//        slowDown = max_speed * slowDownTicks;
//        updateTargets();
//        while ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {
//
//            if (Math.abs(dPosY) < slowDown) {
//                a = dPosY / slowDown;
//            } else {
//                a = 1;
//            }
//            if (Math.abs(dPosX) < slowDown) {
//                b = dPosX / slowDown;
//            } else {
//                b = 1;
//            }
//
//            FWD = a * max_speed;
//            speed = min_speed + a * (max_speed-min_speed);
//            STR = b * max_speed;
//            if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
//                FWD = -FWD;
//            }
//            if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
//                STR = -STR;
//            }
//            turn = Kp*Math.abs(speed)*dHeading;
//
//            telemetry();
//
//            telemetry.update();
//
//            FrontL.setPower(FWD + STR + turn);
//            FrontR.setPower(FWD - STR - turn);
//            BackL.setPower(FWD - STR + turn);
//            BackR.setPower(FWD + STR - turn);
//            updateTargets();
//        }
//        Stop();
//        myOpMode.sleep(100);
//    }
//
//    //TODO: put these first versions in EN, then delete three methods if not necessary
//    public void setFSMDrive(double x, double y, double max_speed) {
//        calibrateEncoders();
//        beginTime = System.currentTimeMillis();
//        heading = current_target_heading;
//        tickY = (int) (y * ourTicksPerCM);
//        tickX = (int) (x * ourTicksPerCM);
//        slowDown = max_speed * 35000;
//        updateTargets();
//        state++;
//    }
//    public void runFSMDrive(double max_speed, double stopTime) {
//        if ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {
//
//            if (Math.abs(dPosY) < slowDown) {
//                a = dPosY / slowDown;
//            } else {
//                a = 1;
//            }
//            if (Math.abs(dPosX) < slowDown) {
//                b = dPosX / slowDown;
//            } else {
//                b = 1;
//            }
//
//            FWD = a * max_speed;
//            speed = min_speed + a * (max_speed - min_speed);
//            STR = b * max_speed;
//            if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
//                FWD = -FWD;
//            }
//            if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
//                STR = -STR;
//            }
//            ROT = Kp * Math.abs(speed) * dHeading;
//
//            telemetry();
//
//            FrontL.setPower(FWD + STR + ROT);
//            FrontR.setPower(FWD - STR - ROT);
//            BackL.setPower(FWD - STR + ROT);
//            BackR.setPower(FWD + STR - ROT);
//            updateTargets();
//        } else {
//            state++;
//        }
//    }
//    public void endFSMDrive() {
//        Stop();
//        myOpMode.sleep(100);
//        state++;
//    }
//
//    //TODO: documentation
//    public void drive(double x, double y){
//        //TODO: calculation for stopTime based on max_speed and dPos
//        drive(x, y, 0.7, 5000);
//    }
//
//    //TODO: documentation
//    public void drive(double x, double y, double stopTime){drive(x, y, 0.7, stopTime);}
//
//    //TODO: documentation
//    //TODO: change FWD, STR, ROT to PID
//    //TODO: global coordinates
//    //TODO: change heading during drive
//    public void drive(double x, double y, double max_speed, double stopTime) {
//        switch (driveState) {
//            case 0:
//                calibrateEncoders();
//                beginTime = System.currentTimeMillis();
//                heading = current_target_heading;
//                tickY = (int) (y * ourTicksPerCM);
//                tickX = (int) (x * ourTicksPerCM);
//                slowDown = max_speed * 35000;
//                updateTargets();
//                driveState++;
//                break;
//            case 1:
//                if ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {
//
//                    if (Math.abs(dPosY) < slowDown) {
//                        a = dPosY / slowDown;
//                    } else {
//                        a = 1;
//                    }
//                    if (Math.abs(dPosX) < slowDown) {
//                        b = dPosX / slowDown;
//                    } else {
//                        b = 1;
//                    }
//
//                    FWD = a * max_speed;
//                    speed = min_speed + a * (max_speed - min_speed);
//                    STR = b * max_speed;
//                    if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
//                        FWD = -FWD;
//                    }
//                    if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
//                        STR = -STR;
//                    }
//                    ROT = Kp * Math.abs(speed) * dHeading;
//
//                    telemetry();
//
//                    //TODO: different motor power algorithm
//                    FrontL.setPower(FWD + STR + ROT);
//                    FrontR.setPower(FWD - STR - ROT);
//                    BackL.setPower(FWD - STR + ROT);
//                    BackR.setPower(FWD + STR - ROT);
//                    updateTargets();
//                } else {
//                    driveState++;
//                }
//                break;
//            case 2:
//                Stop();
//                break;
//        }
//    }
//    //TODO: documentation
//    public void updateTargets(){
//        dHeading = getCurrentHeading() - heading;
//        OdoY_Pos = FrontL.getCurrentPosition();
//        OdoX_Pos = FrontR.getCurrentPosition();
//        TimeElapsed = System.currentTimeMillis() - beginTime;
//        dPosY = tickY - OdoY_Pos;
//        dPosX = tickX - OdoX_Pos;
//        dPos = Math.abs(dPosX) + Math.abs(dPosY);
//    }
//    public void telemetry(){
//        telemetry.addData("tickY", tickY);
//        telemetry.addData("PosY", OdoY_Pos / ourTicksPerCM);
//        telemetry.addData("dPosY", dPosY);
//        telemetry.addData("tickX", tickX);
//        telemetry.addData("PosX", OdoX_Pos / ourTicksPerCM);
//        telemetry.addData("dPosX", dPosX);
//        telemetry.addData("speed", speed);
//        telemetry.addData("a", a);
//        telemetry.addData("dPos", dPos);
//        telemetry.addData("CurrentHeading", getCurrentHeading());
//        telemetry.addData("TargetHeading", heading);
//        telemetry.addData("STR", STR);
//        telemetry.addData("FWD", FWD);
//        telemetry.addData("turn", ROT);
//        telemetry.addData("time", runtime.milliseconds());
//        telemetry.addData("FL power", FrontL.getPower());
//        telemetry.addData("FR power", FrontR.getPower());
//        telemetry.addData("BL power", BackL.getPower());
//        telemetry.addData("BR power", BackR.getPower());
//    }
//    //TODO: documentation
//    public void rotateToHeading(double target_heading, double speed) {
//        target_heading *= -1;
//        switch (driveState) {
//            case 0:
//                dHeading = target_heading - getCurrentHeading();
//                current_target_heading = target_heading;
//                rotateThreshold = 0.5;
//                telemetry.addData("curHeading", getCurrentHeading());
//                telemetry.addData("dHeading",dHeading);
//                driveState++;
//                break;
//            case 1:
//                if (!(Math.abs(dHeading) < rotateThreshold) && myOpMode.opModeIsActive()) {
//                    direction = checkDirection(dHeading);
//
//                    if (Math.abs(dHeading) < (10 * rotateThreshold)) {
//                        speed = 0.2;
//                    }
//
//                    //TODO: different motor power algorithm
//                    FrontL.setPower(-speed * direction);
//                    FrontR.setPower(speed * direction);
//                    BackL.setPower((-speed * direction));
//                    BackR.setPower(speed * direction);
//
//                    dHeading = target_heading - getCurrentHeading();
//                    telemetry.addData("curHeading", getCurrentHeading());
//                    telemetry.addData("dHeading",dHeading);
//                } else {
//                    driveState++;
//                }
//                break;
//            case 2:
//                calibrateEncoders();
//                Stop();
//                break;
//        }
//    }
//
//    //TODO: documentation
//    public void linearAprilTagBackboardCorrection(double offsetYaw, double offsetZ) {
//        calibrateEncoders();
//        double corOffsetZ = (1.27 * offsetZ + 0.0471)*100;
//        double offsetX = Math.atan(Math.toRadians(offsetYaw)) * corOffsetZ;
//        telemetry.addData("offsetX", offsetX);
//        telemetry.addData("corOffsetZ", offsetZ);
//        linearDrive(offsetX,-corOffsetZ);
//    }
//
//    //TODO: documentation
//    //TODO: method that can do this while robotCentric
//    public void AprilTagBackboardCorrection(double offsetYaw, double offsetZ) {
//        calibrateEncoders();
//        double corOffsetZ = (1.27 * offsetZ + 0.0471)*100;
//        double offsetX = Math.atan(Math.toRadians(offsetYaw)) * corOffsetZ;
//        telemetry.addData("offsetX", offsetX);
//        telemetry.addData("corOffsetZ", offsetZ);
//        drive(offsetX,-corOffsetZ);
//    }
//
//    /**
//     * This method is a shorthand for the regular driveY, with the speed and telemetry filled in already.
//     * This makes long strings of auton with similar driveY's easier to write and read.
//     * @param position - How far you want to travel, in centimeters
//     */
//    public void linearDriveY(double position){
//        linearDriveY(position,0.3, 10000);
//    }
//
//    /**
//     * This method is for autonomously driving in the primary axis of motion, which, for us, means
//     * forwards or backwards. You could rename this to driveX as the X-axis is usually the primary
//     * axis for vehicles, but we chose not to. This method does not remember its coordinate system because of the calibrateEncoders, it resets to
//     * zero each time.
//     * @param position - How far you want to travel, in centimeters. A positive position means forward, a negative position means backwards.
//     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
//     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
//     */
//    public void linearDriveY(double position, double speed, double stopTime) {
//        calibrateEncoders();
//        double beginTime = System.currentTimeMillis();
//        double TimeElapsed = System.currentTimeMillis() - beginTime;
//        double turn;
//        double heading = current_target_heading;
//        double OdoY_Pos = FrontL.getCurrentPosition();
//        double tick = (int) (position * ourTicksPerCM);
//        double dPos = tick - OdoY_Pos;
//        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {
//            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
//                speed = -speed;
//            }
//            turn = Kp*Math.abs(speed)*(getCurrentHeading() - heading);
//
//            telemetry.addData("tick", tick);
//            telemetry.addData("PosY", OdoY_Pos/ ourTicksPerCM);
//            telemetry.addData("dPos", dPos);
//            telemetry.addData("speed", speed);
//            telemetry.addData("CurrentHeading", getCurrentHeading());
//            telemetry.addData("TargetHeading", heading);
//            telemetry.addData("time", runtime.milliseconds());
//            telemetry.addData("FL power", FrontL.getPower());
//            telemetry.addData("FR power", FrontR.getPower());
//            telemetry.addData("BL power", BackL.getPower());
//            telemetry.addData("BR power", BackR.getPower());
//            telemetry.update();
//
//            FrontL.setPower(speed + turn);
//            BackL.setPower((speed + turn));
//            BackR.setPower(speed - turn);
//            FrontR.setPower(speed - turn);
//
//            TimeElapsed = System.currentTimeMillis() - beginTime;
//
//            OdoY_Pos = FrontL.getCurrentPosition();
//            dPos = tick - OdoY_Pos;
//        }
//        Stop();
//        myOpMode.sleep(100);
//    }
//
//    /**
//     * This method is a shorthand for the regular driveX, with the speed and telemetry filled in already.
//     * This makes long strings of auton with similar driveX's easier to write and read.
//     * @param position - How far you want to travel, in centimeters
//     */
//    public void linearDriveX(double position){
//        linearDriveX(position,0.3);
//    }
//
//    /**
//     * This method is for autonomously driving in the secondary axis of motion, which, for us, means
//     * to the left or right. You could rename this to driveY as the Y-axis is usually the secondary
//     * axis for vehicles, but we chose not to. This method does not remember its coordinate system because of the calibrateEncoders, it resets to
//     * zero each time.
//     * @param position - How far you want to travel, in centimeters. A positive position means to the right, a negative position means to the left.
//     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
//     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
//     */
//    public void linearDriveX(double position, double speed) {
//        speed = speed * -1;
//        calibrateEncoders();
//        double Kp = 0.03;
//        double turn;
//        double heading = current_target_heading;
//        double tick = (int) (position * ourTicksPerCM);
//        double dPos = tick - OdoX_Pos;
//        while (!((Math.abs(dPos) < threshold)) && myOpMode.opModeIsActive()) {
//            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
//                speed = -speed;
//            }
//            turn = Kp*Math.abs(speed)*(getCurrentHeading() - heading);
//
//            telemetry.addData("tick", tick);
//            telemetry.addData("PosX", OdoX_Pos/ ourTicksPerCM);
//            telemetry.addData("dPos", dPos);
//            telemetry.addData("speed", speed);
//            telemetry.addData("FL power", FrontL.getPower());
//            telemetry.addData("FR power", FrontR.getPower());
//            telemetry.addData("BL power", BackL.getPower());
//            telemetry.addData("BR power", BackR.getPower());
//            telemetry.update();
//
//            FrontL.setPower(-speed + turn);
//            FrontR.setPower(speed - turn);
//            BackL.setPower(speed + turn);
//            BackR.setPower(-speed - turn);
//
//            OdoX_Pos = FrontR.getCurrentPosition();
//            dPos = tick - OdoX_Pos;
//        }
//        Stop();
//        myOpMode.sleep(100);
//    }
//
//    /**
//     * This method is a shorthand for the regular rotateToHeading, with the speed and telemetry filled in already.
//     * This makes long strings of auton with similar rotateToHeading's easier to write and read.
//     * @param target_heading - To which angle you want to turn, in degrees. This means that with two successive
//     *                       rotateToHeading(90)'s, the second rotateToHeading is useless, as you are at that heading already.
//     */
//    public void linearRotateToHeading(double target_heading){
//        linearRotateToHeading(target_heading,0.4);
//    }
//    /**
//     * This method is for autonomously driving turning around the Z-axis or yaw. It might make sense to reverse target_heading,
//     * so that positive is counterclockwise, because that is how degrees usually work. We preferred clockwise as it made more intuitive sense.
//     * @param target_heading - To which angle you want to turn, in degrees. This means that with two successive
//     *                       rotateToHeading(90)'s, the second rotateToHeading is useless, as you are at that heading already.
//     *                       You could change this by making the parameter the dHeading (delta Heading). A positive target_heading means clockwise,
//     *                       a negative heading means counterclockwise. It's interval is therefore [-180,180]
//     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
//     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
//     */
//    public void linearRotateToHeading(double target_heading, double speed) {
//        target_heading *= -1;
//        double current_heading = getCurrentHeading();
//        double dHeading = target_heading - current_heading;
//        double direction;
//        double margin = 0.5;
//        telemetry.addData("curHeading", current_heading);
//        telemetry.addData("dHeading",dHeading);
//        telemetry.update();
//        while (!(Math.abs(dHeading) < margin) && myOpMode.opModeIsActive()) {
//            direction = checkDirection(dHeading);
//
//            FrontL.setPower(-speed * direction);
//            FrontR.setPower(speed * direction);
//            BackL.setPower((-speed * direction));
//            BackR.setPower(speed * direction);
//
//            current_heading = getCurrentHeading();
//            dHeading = target_heading - current_heading;
//            if(dHeading < 10 * margin) {
//                speed = 0.2;
//            }
//            telemetry.addData("curHeading", current_heading);
//            telemetry.addData("dHeading",dHeading);
//            telemetry.update();
//        }
//        calibrateEncoders();
//        Stop();
//        current_target_heading = target_heading;
//    }
//
//    /**
//     * This method checks if the parameter is positive or negative, which is used to check if the
//     * rotateToHeading needs to rotate clockwise or counterclockwise.
//     * @param val - Interval can be anything.
//     * @return - Returns -1 if a negative number is inputted and 1 if a positive number is inputted.
//     */
//    int checkDirection(double val){
//        if (val < 0)
//            return -1;
//        else return 1;
//    }
//
//    /**
//     * This method is used at the end of other methods to ensure all motors are at zero power. The alternative,
//     * stopping with commands, might result in never stopping or a large distance at the end of each method,
//     * during which the robot slowly slows to a halt.
//     */
//    public void Stop(){
//        FrontL.setPower(0);
//        FrontR.setPower(0);
//        BackL.setPower(0);
//        BackR.setPower(0);
//
//        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    /**
//     * This method uses the 2023 universal IMU code (works for both BHI260AP and BNO055 chips) and
//     * the Control Hub's or Expansion Hub's IMU chip to figure out the robots heading. This value
//     * does not reset when switching from Autonomous to Tele-Op opmodes. We use this method for autonomous
//     * because we have more intuition with degrees.
//     * @return - Returns the robots current heading, in degrees.
//     */
//    public double getCurrentHeading() {
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        return (orientation.getYaw(AngleUnit.DEGREES));
//    }
//
//    /**
//     * ResetYaw is one of the new IMU methods, and it resets the yaw of the robot. When implemented correctly,
//     * yaw is the only rotate axis you want to change.
//     */
//    public void resetYaw() {
//        imu.resetYaw();
//    }
//
//    /**
//     * This method resets the encoders to a new zero position, so the next method starts from position zero again.
//     */
//    public void calibrateEncoders() {
//        FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//}