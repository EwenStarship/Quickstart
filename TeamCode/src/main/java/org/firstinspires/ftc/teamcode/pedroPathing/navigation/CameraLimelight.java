package org.firstinspires.ftc.teamcode.pedroPathing.navigation;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CameraLimelight {

    private Limelight3A limelight;

    // --- Buffer circulaire pour filtrer la distance ---
    private double[] buffer = new double[3];
    private int index = 0;
    private int count = 0;

    // Constructeur vide (comme HuskyLens)
    public CameraLimelight() {}

    // Initialisation (comme HuskyLens.init())
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    /** Retourne le dernier résultat brut */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /** Retourne TX ou null si pas de tag */
    public Double getTx() {
        LLResult result = getResult();
        if (result == null || !result.isValid()) return null;
        return result.getTx();
    }

    /** Retourne TA ou null si pas de tag */
    public Double getTa() {
        LLResult result = getResult();
        if (result == null || !result.isValid()) return null;
        return result.getTa();
    }

    /** Distance rectifiée sur sur TA */
    public Double getDistance() {
        Double ta = getTa();
        if (ta == null || ta <= 0) return null;

        return 186.6697 * Math.pow(ta, -0.6873269);
    }


    /** Distance filtrée (comme HuskyLens.getDistanceFiableCm()) */
    public double getDistanceFiableCm() {

        Double dist = getDistance();
        if (dist == null || dist < 0) return -1;


        // Ajout dans buffer circulaire
        buffer[index] = dist;
        index = (index + 1) % 3;
        if (count < 3) count++;

        if (count < 3) return -1;

        // Recherche d’un groupe cohérent
        for (int i = 0; i < 3; i++) {
            int c = 1;
            double somme = buffer[i];

            for (int j = 0; j < 3; j++) {
                if (i != j && Math.abs(buffer[j] - buffer[i]) <= 5.0) {
                    somme += buffer[j];
                    c++;
                }
            }

            if (c >= 3) {
                return somme / c;
            }
        }

        return -1;
    }
    public boolean hasTag() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }

}


