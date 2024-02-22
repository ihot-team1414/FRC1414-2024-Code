package frc.util;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    ArrayList<Object[]> notes = new ArrayList<Object[]>();
    
    

    int numEntries = 5;   


    double[] txEntries = new double[5];
    double[] tyEntries = new double[5];
    double[] taEntries = new double[5];

    public void Periodic(){
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
 
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //add values to entries arrays
        for (int i = 0; i < numEntries-2; i++){
            txEntries[i+1] = txEntries[i];
        }
        txEntries[0] = x;

        for (int i = 0; i < numEntries-2; i++){
            tyEntries[i+1] = tyEntries[i];
        }
        tyEntries[0] = y;

        for (int i = 0; i < numEntries-2; i++){
            taEntries[i+1] = taEntries[i];
        }
        taEntries[0] = area;
        
        Object[] observation = new Object[4];

        observation[0] = tx;
        observation[1] = ty;
        observation[2] = timestamp;
        observation[3] = confidence;

        //check if there is already a note in notes that has similar position to observation. If there is one, new x,y for that note is weighted average of the old and new x,y based off of confidence(distance).
        //if not...{
            

    }
}

//timestamp to throw awaya older entries, with new entries find the weighted average
//We need x, y, time, confidence