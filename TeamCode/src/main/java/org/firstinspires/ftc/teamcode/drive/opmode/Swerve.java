package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name="Driver - Swerve", group="Linear Opmode")
@Disabled
public class Swerve extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx topLeft  = null;
    private DcMotorEx botLeft  = null;
    private DcMotorEx topRight = null;
    private DcMotorEx botRight = null;

    final static double countPerRev   = 528.65;
    final static double radPerRev     = 2*Math.PI;
    final static double countPerRad   = countPerRev/radPerRev;
    final double joystickDeadzone     = 0.1;
    final double posDeadzone          = 20;
    public static double driveSpeed    = 0.2;
    public static double deltaVSpeed   = 0.1;
    double direction                  = 0;
    double currentRad                 = 0;
    double optimumRad                 = 0;
    double targetVelocity             = 0;
    double targetAngle                = 0;
    double currentCount               = 0;

    public void setRadAndVel (double yInput, double xInput){
        double rad = Math.atan2(Math.abs(yInput), xInput);
        if (yInput < 0){
            rad = (2*Math.PI) - rad;
        }
        targetAngle = rad * (180/Math.PI);
        setOptimumRad(rad, currentRad);
        double velocity = Math.sqrt(Math.abs(yInput*yInput)+Math.abs(xInput*xInput));
        targetVelocity = velocity * direction;
    }

    public void setOptimumRad (double target, double current){
        double optimum =  target - current;
        if (optimum >  Math.PI/2 | optimum < -1*(Math.PI/2)){
            optimum = target + current;
            direction = -1;
        }else {
            direction = 1;
        }
        optimumRad = optimum;
    }

    public double leftAngleCorrection (double angle) {
        double tlPow = Math.abs(topLeft.getCurrentPosition() % countPerRev);
        double blPow = Math.abs(botLeft.getCurrentPosition() % countPerRev);
        double tarCount = angle * countPerRad;
        currentCount = tlPow - blPow;
        currentRad = currentCount/countPerRad;
        return deltaVSpeed * dzPos(tarCount - currentCount);
    }
    /*
        public double rightAngleCorrection (double angle) {
            double trPow = Math.abs(topRight.getCurrentPosition() % countPerRev);
            double brPow = Math.abs(botRight.getCurrentPosition() % countPerRev);
            double radCount = angle * countPerRad;
            currentRad = trPow - brPow;
            return deltaVSpeed * dzPos(radCount - currentRad);
        }
    */
    public double dzJoy(double input){
        double output;
        if(Math.abs(input) < joystickDeadzone) {
            output = 0.0;
        }else {
            output = input;
        }
        return output;
    }

    public double dzPos(double input){
        double output;
        if(Math.abs(input) < posDeadzone) {
            output = 0.0;
        }else {
            output = input;
        }
        return output;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        topLeft   = hardwareMap.get(DcMotorEx.class, "topLeft");
        botLeft   = hardwareMap.get(DcMotorEx.class, "botLeft");
        topRight  = hardwareMap.get(DcMotorEx.class, "topRight");
        botRight  = hardwareMap.get(DcMotorEx.class, "botRight");

        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        botRight.setDirection(DcMotor.Direction.FORWARD);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for Play
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double TLVelocity;
            double BLVelocity;
            double TRVelocity;
            double BRVelocity;

            setRadAndVel(dzJoy(-gamepad1.left_stick_y),dzJoy(gamepad1.left_stick_x));

            double VelCubed = targetVelocity; // * targetVelocity * targetVelocity;

            TLVelocity   = Range.clip(VelCubed + leftAngleCorrection(optimumRad), -1.0, 1.0) ;
            BLVelocity   = Range.clip(VelCubed - leftAngleCorrection(optimumRad), -1.0, 1.0) ;
            TRVelocity   = Range.clip(VelCubed + leftAngleCorrection(optimumRad), -1.0, 1.0) ;
            BRVelocity   = Range.clip(VelCubed - leftAngleCorrection(optimumRad), -1.0, 1.0) ;

            topLeft.setVelocity(TLVelocity * driveSpeed);
            botLeft.setVelocity(BLVelocity * driveSpeed);
//            topRight.setVelocity(TRVelocity * driveSpeed);
//            botRight.setVelocity(BRVelocity * driveSpeed);

            telemetry.addData("Top Left Motor Power", TLVelocity);
            telemetry.addData("Bot Left Motor Power", BLVelocity);
            telemetry.addData("Target Power", VelCubed);
            telemetry.addData("Target angle", targetAngle);
            telemetry.addData("Count difference", currentCount );
            telemetry.update();
        }
    }
}