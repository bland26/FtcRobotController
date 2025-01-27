package org.firstinspires.ftc.teamcode.drive.hive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Astro Driver", group="Hive")
@Config
@Disabled

//@Disabled
public class HiveTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    private DcMotor lift = null;
    private DcMotor intake = null;

    private DcMotor climb = null;
    private CRServo outtake = null;

    private  TouchSensor limitDown;

    private TouchSensor limitClimb;

    private Servo claw = null;
    private Servo drone = null;

    final static double clawStart = 0.1;
    public static double clawMin = 0.0;
    public static double clawMax = 0.4;
    ;
    public static double clawSpeed = 0.01;
    public double clawPosition = 0.0;

    static double droneStart = 0;

    public static double dronePosition = 1.0;

    //TODO Decide names for and declare extra motors. (Top intake, bottom intake, lift)
    //TODO Decide names for and declare servos.


    public static double driveSpeed = 1.0;

    public static double liftSpeed = 1.0;

    public static double intakeSpeed = 0.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        climb = hardwareMap.get(DcMotor.class,"climb");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        limitDown = hardwareMap.get(TouchSensor.class, "limitDown");
        limitClimb = hardwareMap.get(TouchSensor.class, "limitClimb");
        claw = hardwareMap.get(Servo.class, "claw");
        drone = hardwareMap.get(Servo.class, "drone");

        //TODO initilize new motors that were added

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(CRServo.Direction.FORWARD);
        climb.setDirection(DcMotor.Direction.REVERSE);
        claw.setPosition(clawPosition);
        drone.setPosition(dronePosition);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO set new motor directions

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftRearPower;
        double rightRearPower;
        double leftFrontPower;
        double rightFrontPower;
        double liftPower;
        double climbPower;
        double outtakePower = 0;
        boolean scoreCon = false;



        //TODO initilize New Motor power variables



        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        final double DEADZONE = 0.1;

        double drive = -gamepad1.left_stick_y;
        if (Math.abs(drive) < DEADZONE) {
            drive = 0;
        }
        double driveCubed = drive * drive * drive;

        double strafe = gamepad1.left_stick_x;
        if (Math.abs(strafe) < DEADZONE) {
            strafe = 0;
        }
        double strafeCubed = strafe * strafe * strafe;

        double spin = gamepad1.right_stick_x;
        if (Math.abs(spin) < DEADZONE) {
            spin = 0;
        }
        double spinCubed = spin * spin * spin;

        double liftInput = gamepad2.left_stick_y;
        if (Math.abs(liftInput) < DEADZONE) {
            liftInput = 0;
        }
        double liftCubed = liftInput * liftInput * liftInput;

        double intakeInput = gamepad2.right_stick_y;
        if (Math.abs(intakeInput) < DEADZONE) {
            intakeInput = 0;}
        double intakeCubed = intakeInput* intakeInput* intakeInput;

        boolean outtakeInput = gamepad2.a;
        if (outtakeInput){
            outtakePower=1;
        }else {
            outtakePower=0;
        }

        boolean outtakeInputR = gamepad2.b;
        if (outtakeInputR){
            outtakePower = -1;
        }

        boolean droneInput = gamepad2.y;
        if(droneInput){
            dronePosition = 0;
        }
        boolean droneInputSet = gamepad2.x;
        if(droneInputSet){
            dronePosition = 1;
        }

        if (gamepad2.right_trigger > 0 && clawPosition < clawMax)
            clawPosition += clawSpeed;
        if (gamepad2.left_trigger > 0 && clawPosition >= clawMin)
            clawPosition -= clawSpeed;

        boolean climbInput = gamepad1.right_bumper;
        if (climbInput && !limitClimb.isPressed()){
            climbPower=1;
        }else {
            climbPower=0;
        }

        boolean climbInputR = gamepad1.left_bumper;
        if (climbInputR){
            climbPower = -1;
        }

        float scoreConInput = gamepad1.right_trigger;
        if (scoreConInput > 0){
            scoreCon = true;
        }









        //TODO create methods for new motors


        if (!scoreCon) {
            leftRearPower = Range.clip(driveCubed + spinCubed - strafeCubed, -1.0, 1.0);
            rightRearPower = Range.clip(driveCubed - spinCubed + strafeCubed, -1.0, 1.0);
            leftFrontPower = Range.clip(driveCubed + spinCubed + strafeCubed, -1.0, 1.0);
            rightFrontPower = Range.clip(driveCubed - spinCubed - strafeCubed, -1.0, 1.0);
            liftPower = Range.clip(liftCubed, -1.0, 1.0);
        }else {
            leftRearPower = Range.clip((driveCubed) + spinCubed - (strafeCubed), -0.5, 0.5);
            rightRearPower = Range.clip((driveCubed)- spinCubed + (strafeCubed), -0.5, 0.5);
            leftFrontPower = Range.clip((driveCubed) + spinCubed + (strafeCubed), -0.5, 0.5);
            rightFrontPower = Range.clip((driveCubed) - spinCubed - (strafeCubed), -0.5, 0.5);
            liftPower = Range.clip(liftCubed, -1.0, 1.0);
        }



        clawPosition = Range.clip(clawPosition,clawMin,clawMax);
        claw.setPosition(clawPosition);
        drone.setPosition(dronePosition);

        // Send calculated power to wheels
        leftRear.setPower(leftRearPower * driveSpeed);
        rightRear.setPower(rightRearPower * driveSpeed);
        leftFront.setPower(leftFrontPower * driveSpeed);
        rightFront.setPower(rightFrontPower * driveSpeed);
        intake.setPower(intakeCubed * intakeSpeed);
        climb.setPower(climbPower);
        outtake.setPower(outtakePower);

        if (liftPower > 0 && limitDown.isPressed()) {
            lift.setPower(0);
        } else {
            lift.setPower(liftPower * liftSpeed);
        }



        //TODO set new motor power


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftRearPower, rightRearPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

