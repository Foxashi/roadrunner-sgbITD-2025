package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
//@TeleOp


public class AlbastruDreapta extends LinearOpMode {

    DcMotor fdr, sdr, fst, sst; // sasiu

    DcMotor gdr, gst; // glisiera

    Servo brat, cleste;

    public void fata(double power) {
        fdr.setPower(power);
        sdr.setPower(power);
        fst.setPower(power);
        sst.setPower(power);
    }
    public void spate(double power) {
        fdr.setPower(-power);
        sdr.setPower(-power);
        fst.setPower(-power);
        sst.setPower(-power);
    }

    public void dreapta(double power){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(power);
        sst.setPower(-power);
    }
    public void stanga(double power){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(-power);
        sst.setPower(power);
    }
    public void rotirestanga(double power){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(power);
        sst.setPower(-power);
    }
    public void rotiredreapta(double power){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(-power);
        sst.setPower(power);
    }

    public void runOpMode() throws InterruptedException {


        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setDirection(DcMotorSimple.Direction.REVERSE);
        fdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setDirection(DcMotorSimple.Direction.REVERSE);
        sdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        fst = hardwareMap.dcMotor.get("fst");
        fst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        gdr = hardwareMap.dcMotor.get("gdr");
        gdr.setDirection(DcMotorSimple.Direction.REVERSE);
        gdr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        gdr.setMode(RunMode.STOP_AND_RESET_ENCODER);


        gst = hardwareMap.dcMotor.get("gst");
        gst.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        gst.setMode(RunMode.STOP_AND_RESET_ENCODER);

        brat = hardwareMap.servo.get("brat");
        brat.setPosition(0.55);

        cleste = hardwareMap.servo.get("cleste");
        cleste.setPosition(0.45);



        waitForStart();

        dreapta(0.6);
        sleep(1000);

    }
}