package strategy;


import java.util.LinkedList;
import java.util.Queue;

// This class will contain variables which need to be preserved between dynamic world changes. Needs to be updated with each dynamic world reinitialisation
 public class StaticVariables {

    public static boolean haveBall = false;
    public static void reset(){
        haveBall = false;
    }

}
