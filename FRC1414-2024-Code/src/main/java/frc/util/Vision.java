package frc.util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;

public class Vision{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //Array list
    
    

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



        //post to smart dashboard periodically
        /*
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        */
    }
}

//timestamp to throw awaya older entries, with new entries find the weighted average
//We need x, y, time, confidence