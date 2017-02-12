package vision.spotAnalysis.recursiveSpotAnalysis;

import vision.colorAnalysis.SDPColor;
import vision.colorAnalysis.SDPColorInstance;
import vision.colorAnalysis.SDPColors;
import vision.constants.Constants;
import vision.gui.Preview;
import vision.spotAnalysis.SpotAnalysisBase;
import vision.spotAnalysis.approximatedSpotAnalysis.RegionFinder;
import vision.spotAnalysis.approximatedSpotAnalysis.Spot;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

import static vision.tools.ImageTools.rgbToHsv;
import static vision.tools.ImageTools.rbgToNormalized;

/**
 * Created by Simon Rovder
 */
public class RecursiveSpotAnalysis extends SpotAnalysisBase{

    private int[] rgb;
    private int[] rgb_normalized;
    private float[] hsv;
    private SDPColor[] found;


    public RecursiveSpotAnalysis(){
        super();
        // Have arrays of 4 times the size for the inputs\
        // (for red, green, blue, alpha OR hue, saturation, value, alpha)
        this.rgb   = new int[4* Constants.INPUT_WIDTH*Constants.INPUT_HEIGHT];
        this.hsv   = new float[4*Constants.INPUT_WIDTH*Constants.INPUT_HEIGHT];
        //this.rgb_normalized   = new int[4*Constants.INPUT_WIDTH*Constants.INPUT_HEIGHT];

        // array to keep track of visited spots
        this.found = new SDPColor[Constants.INPUT_WIDTH*Constants.INPUT_HEIGHT];
    }

    private int getIndex(int x, int y){
        return y*Constants.INPUT_WIDTH*3 + x*3;
    }

    private void processPixel(int x, int y, SDPColorInstance sdpColorInstance, XYCumulativeAverage average, int maxDepth){
        if(maxDepth <= 0 || x < 0 || x >= Constants.INPUT_WIDTH || y < 0 || y >= Constants.INPUT_HEIGHT) return;
        int i = getIndex(x, y);
        if(this.found[i/3] == sdpColorInstance.sdpColor) return;
        if(sdpColorInstance.isColor(this.hsv[i], this.hsv[i + 1], this.hsv[i + 2])){
            average.addPoint(x, y);
            this.found[i/3] = sdpColorInstance.sdpColor;
            this.processPixel(x-1,y, sdpColorInstance, average, maxDepth - 1);
            this.processPixel(x+1,y, sdpColorInstance, average, maxDepth - 1);
            this.processPixel(x,y+1, sdpColorInstance, average, maxDepth - 1);
            this.processPixel(x,y-1, sdpColorInstance, average, maxDepth - 1);
            Graphics g = Preview.getImageGraphics();
            if(g != null && sdpColorInstance.isVisible()){
                g.setColor(Color.WHITE);
                g.drawRect(x,y,1,1);
            }
        }
    }


    @Override
    public void nextFrame(BufferedImage image, long time) {


        Raster raster = image.getData();

        /*
         * SDP2017NOTE
         * This line right here, right below is the reason our vision system is real time. We fetch the
         * rgb values of the Raster into a preallocated array this.rgb, without allocating more memory.
         * We recycle the memory, so garbage collection is never called.
         */
        raster.getPixels(0, 0, Constants.INPUT_WIDTH, Constants.INPUT_HEIGHT, this.rgb);

        rgbToHsv(this.rgb, this.hsv);
        //rbgToNormalized(this.rgb, this.rgb_normalized);
        //rgbToHsv(this.rgb_normalized, this.hsv);

        HashMap<SDPColor, ArrayList<Spot>> spots = new HashMap<SDPColor, ArrayList<Spot>>();
        spots.put(SDPColor.YELLOW, new ArrayList<Spot>());
        spots.put(SDPColor.GREEN, new ArrayList<Spot>());
        spots.put(SDPColor.BLUE, new ArrayList<Spot>());
        spots.put(SDPColor.PINK, new ArrayList<Spot>());
        spots.put(SDPColor._BALL, new ArrayList<Spot>());
        //for(SDPColor c : SDPColor.values()){
            /*
            if(c == SDPColor.GREEN_1 || c == SDPColor.GREEN_2){
                //spots.put(c, new ArrayList<Spot>());
                spots.put(SDPColor.GREEN, new ArrayList<Spot>());
                System.out.println("green if");
            } else if(c == SDPColor.BLUE_1 || c == SDPColor.BLUE_2){
                spots.put(SDPColor.BLUE, new ArrayList<Spot>());
                System.out.println("blue if");
            } else if(c == SDPColor.PINK_1 || c == SDPColor.PINK_2){
                spots.put(SDPColor.PINK, new ArrayList<Spot>());
                System.out.println("pink if");
            } else if(c == SDPColor.YELLOW_1 || c == SDPColor.YELLOW_2){
                spots.put(SDPColor.YELLOW, new ArrayList<Spot>());
                System.out.println("yellow if");
            } else {
                spots.put(c, new ArrayList<Spot>());
                System.out.println("else");
            }
            */
            //spots.put(c, new ArrayList<Spot>());
        //}

        XYCumulativeAverage average = new XYCumulativeAverage();
        SDPColorInstance colorInstance;
        for(int i = 0 ; i < Constants.INPUT_HEIGHT * Constants.INPUT_WIDTH; i++){
            this.found[i] = null;
        }
        for(SDPColor color : SDPColor.values()){
            /*
            if(color == SDPColor.GREEN_1 || color == SDPColor.GREEN_2){
                //spots.put(c, new ArrayList<Spot>());
                color = SDPColor.GREEN;
            } else if(color == SDPColor.BLUE_1 || color == SDPColor.BLUE_2){
                color = SDPColor.BLUE;
            } else if(color == SDPColor.PINK_1 || color == SDPColor.PINK_2){
                color = SDPColor.PINK;
            } else if(color == SDPColor.YELLOW_1 || color == SDPColor.YELLOW_2){
                color = SDPColor.YELLOW;
            }
            */
            colorInstance = SDPColors.colors.get(color);
            for(int y = 0; y < Constants.INPUT_HEIGHT; y++){
                for(int x = 0; x < Constants.INPUT_WIDTH; x++){
                    this.processPixel(x, y, colorInstance, average, 200);
                    if(average.getCount() > 5){
                        SDPColor colorNew;
                        if(color == SDPColor.GREEN_1 || color == SDPColor.GREEN_2){
                            //spots.put(c, new ArrayList<Spot>());
                            colorNew = SDPColor.GREEN;
                        } else if(color == SDPColor.BLUE_1 || color == SDPColor.BLUE_2){
                            colorNew = SDPColor.BLUE;
                        } else if(color == SDPColor.PINK_1 || color == SDPColor.PINK_2){
                            colorNew = SDPColor.PINK;
                        } else if(color == SDPColor.YELLOW_1 || color == SDPColor.YELLOW_2){
                            colorNew = SDPColor.YELLOW;
                        } else{
                            colorNew = color;
                        }
                        spots.get(colorNew).add(new Spot(average.getXAverage(), average.getYAverage(), average.getCount(), colorNew));
                    }
                    average.reset();
                }
            }
            if(color == SDPColor.GREEN_1 || color == SDPColor.GREEN_2){
                //spots.put(c, new ArrayList<Spot>());
                Collections.sort(spots.get(SDPColor.GREEN));
            } else if(color == SDPColor.BLUE_1 || color == SDPColor.BLUE_2){
                Collections.sort(spots.get(SDPColor.BLUE));
            } else if(color == SDPColor.PINK_1 || color == SDPColor.PINK_2){
                Collections.sort(spots.get(SDPColor.PINK));
            } else if(color == SDPColor.YELLOW_1 || color == SDPColor.YELLOW_2){
                Collections.sort(spots.get(SDPColor.YELLOW));
            } else{
                Collections.sort(spots.get(color));
            }
            //Collections.sort(spots.get(color));
        }
        this.informListeners(spots, time);
        Preview.flushToLabel();

    }
}
