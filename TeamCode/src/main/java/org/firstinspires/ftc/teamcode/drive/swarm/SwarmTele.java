/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.swarm;

import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="SwarmTele", group="Teleop")

public class SwarmTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare end-effector members
    private CRServo intake = null;
    //private Ser = null;
    private DcMotor extension = null;
    private DcMotorEx pivot = null;

    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -1.0;
    private double INTAKE_OFF_POWER = 0.0;

    private double intakePower ;


    private double EXTENSION_OUT_POWER = 1.0;
    private double EXTENSION_IN_POWER = -1.0;

    private int pivot_target_pos;
    private int pivot_home_pos;

    private double PIVOT_UP_POWER = 0.25;
    private double PIVOT_DOWN_POWER = -0.0125;
    private double PIVOT_HOLD_POWER = 0.001;
    private enum PivotModes {UP, HOLD, DOWN};
    private PivotModes pivotMode;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(CRServo.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        // TODO: Make sure all motors are facing the correct direction. Go one at a time.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(CRServo.Direction.FORWARD); // Forward should INTAKE.
        extension.setDirection(DcMotor.Direction.REVERSE); // Forward should EXTEND.
        pivot.setDirection(DcMotor.Direction.REVERSE); // Forward should pivot UP, or away from the stowed position.

//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_home_pos = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Tell the driver that initialization is complete.
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        pivotMode = PivotModes.HOLD;
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double max;

        // COLLECT INPUTS
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

//        boolean intakeInButton = gamepad1.a;
//        boolean intakeOutButton = gamepad1.b;
//        boolean intakeOffButton = gamepad1.x;
//        // This conditional reduces ambiguity when multiple buttons are pressed.
//        if (intakeInButton && intakeOutButton) {
//            intakeInButton = false;
//        } else if (intakeOffButton && (intakeInButton || intakeOutButton)) {
//            intakeInButton = intakeOutButton = false;
//        }

        float intakeInput = gamepad2.right_trigger;
        if (intakeInput > 0){
            intakePower=1;
        }else {
            intakePower=0;
        }

        float intakeInputR = gamepad2.left_trigger;
        if (intakeInputR > 0){
            intakePower = -1;
        }

        boolean extensionOutButton = gamepad2.right_bumper;
        boolean extensionInButton = gamepad2.left_bumper;
        if (extensionOutButton && extensionInButton) {
            extensionOutButton = false;
        }

//        boolean pivotUpButton = gamepad2.right_bumper;
//        boolean pivotDownButton = gamepad2.right_trigger > 0.2;
//        if (pivotUpButton && pivotDownButton) {
//            pivotUpButton = false;
//        }

        double pivotInput = -gamepad2.right_stick_y;
        if (Math.abs(pivotInput) < 0.1) {
            pivotInput = 0;
        }
        double pivotCubed = pivotInput * pivotInput * pivotInput;

        // DRIVE CODE
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // INTAKE CODE
//        if (intakeInButton) {
//            intakePower = INTAKE_IN_POWER;
//        } else if (intakeOutButton) {
//            intakePower = INTAKE_OUT_POWER;
//        } else if (intakeOffButton) {
//            intakePower = 0.0;
//        }

        // EXTENSION CODE
        double extensionPower;
        if (extensionOutButton) {
            extensionPower = EXTENSION_OUT_POWER;
        } else if (extensionInButton) {
            extensionPower = EXTENSION_IN_POWER;
        } else {
            extensionPower = 0;
        }

        // Determine pivot mode
//        if (pivotUpButton) {
//            pivotMode = PivotModes.UP;
//            pivot_target_pos += 5;
//        } else if (pivotDownButton) {
//            pivotMode = PivotModes.DOWN;
//            pivot_target_pos -= 5;
//        } else {
//            pivotMode = PivotModes.HOLD;
//        }

        // Make sure that motor is in the correct control mode.
        // If there is a mismatch, we are transferring into that mode.
        // If we are transferring into HOLD mode, set the target hold position.
//        if ((pivotMode == PivotModes.UP || pivotMode == PivotModes.DOWN) && (pivot.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)) {
//            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        } else if ((pivotMode == PivotModes.HOLD) && (pivot.getMode() != DcMotor.RunMode.RUN_TO_POSITION)) {
//            pivot.setTargetPosition(pivot.getCurrentPosition());
//            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

//        double pivotPower;
//        if (pivotMode == PivotModes.UP) {
//            pivotPower = PIVOT_UP_POWER;
//        } else if (pivotMode == PivotModes.DOWN) {
//            pivotPower = PIVOT_DOWN_POWER;
//        } else {
//            pivotPower = PIVOT_HOLD_POWER;
//        }


        // WRITE EFFECTORS
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


        intake.setPower(intakePower);
        extension.setPower(extensionPower);
        //pivot.setTargetPosition(pivot_target_pos);
        pivot.setPower(pivotCubed);

        String pivot_mode_str;
        if (pivotMode == PivotModes.UP) {
            pivot_mode_str = "UP";
        } else if (pivotMode == PivotModes.DOWN) {
            pivot_mode_str = "DOWN";
        } else {
            pivot_mode_str = "HOLD";
        }
        // UPDATE TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Intake", "%%4.2f", intakePower);
        telemetry.addData("Extension", "%4.2f", extension.getPower());
        telemetry.addData("Pivot Current/Target/power", "%d, %d, %4.2f", pivot.getCurrentPosition(), pivot.getTargetPosition(),pivot.getPower());
        telemetry.addData("Pivot MODE", "%s", pivot_mode_str);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
