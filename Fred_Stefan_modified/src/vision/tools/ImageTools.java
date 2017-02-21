package vision.tools;

import vision.constants.Constants;

import java.awt.*;

/**
 * Created by Simon Rovder
 */
public class ImageTools {

    private static float[] dummyHSV = {0,0,0,0};

    public static void rgbToHsv(int[] original, float[] target){
        for(int i = 0; i < Constants.INPUT_WIDTH*Constants.INPUT_HEIGHT; i++){
            Color.RGBtoHSB(original[3*i], original[3*i + 1], original[3*i + 2], dummyHSV);
            target[i*3]     = dummyHSV[0];
            target[i*3 + 1] = dummyHSV[1];
            target[i*3 + 2] = dummyHSV[2];
        }
    }

    public static void rbgToNormalized(int[] original, int[] target) {
        int sum = 0;
        float temp;
        for (int i = 0; i < Constants.INPUT_WIDTH * Constants.INPUT_HEIGHT; i++) {
            Color.RGBtoHSB(original[3 * i], original[3 * i + 1], original[3 * i + 2], dummyHSV);
            sum = original[3 * i] + original[3 * i + 1] + original[3 * i + 2];
            if (sum == 0) {
                target[i * 3] = 0;
                target[i * 3 + 1] = 0;
                target[i * 3 + 2] = 0;
            } else {
                temp = (target[i * 3] / sum) * 255;
                target[i * 3] = Math.round(temp);
                temp = (target[i * 3 + 1] / sum) * 255;
                target[i * 3 + 1] = Math.round(temp);
                temp = (target[i * 3 + 2] / sum) * 255;
                target[i * 3 + 2] = Math.round(temp);
            }
        }
    }
}
