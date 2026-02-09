package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name="Limelight", group="debug")
public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;
    SpinTurret spinTurret;
    boolean autoAimActive = false;
    boolean isBlueAlliance = true; // ou false selon le match

    public double distanceFromTa(double ta) {
        if (ta <= 0 || Double.isNaN(ta)) return -1;
        return 186.6697 * Math.pow(ta, -0.6873269);
    }

    @Override
    public void init() {
        spinTurret = new SpinTurret();
        spinTurret.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0); // pipeline AprilTag
    }

    @Override
    public void start() {
        limelight.start();
    }


    @Override
    public void loop() {

        spinTurret.update();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx();
            double ta = result.getTa();
            double distance = distanceFromTa(ta);

            telemetry.addData("Tx", tx);
            telemetry.addData("TA", ta);
            telemetry.addData("Distance", distance);
        }else{
            telemetry.addData("Tx", 0);
            telemetry.addData("TA", 0);
            telemetry.addData("Distance", 0);
            autoAimActive = false;
            spinTurret.stopTourelle();
        }
        if (gamepad1.x) {
            autoAimActive = true;   // on active l’auto-aim
        }

// Définir l’offset selon l’alliance
// +2.0 = côté bleu
// -2.0 = côté rouge
        double offset = isBlueAlliance ? 0.0 : -0.0;

        if (autoAimActive && result != null && result.isValid()) {

            double tx = result.getTx();

            // Appel avec offset
            spinTurret.tournerAvecTxoffset(tx, offset);

            // Si on est centré → on coupe l’auto-aim
            if (Math.abs(tx - offset) <= 1.0) {
                autoAimActive = false;
                spinTurret.stopTourelle();
            }

            telemetry.addData("Tx", tx);
            telemetry.addData("Offset", offset);
        }


    }
}


