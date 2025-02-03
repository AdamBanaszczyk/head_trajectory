using System.IO;
using UnityEngine;

public class PoseCollector
{
    private GameObject objectoToTrack;
    private string buffer = "";

    private System.IO.StreamWriter writer;
    private string fileName;
    private string destination;
    private string filePath;
    private string logPrefix = "[pose_collector]";
    
    public PoseCollector(string gameObjectName)
    {
        objectoToTrack = GameObject.Find(gameObjectName);

        fileName = $"{System.DateTime.Now.ToString("HH-mm-ss")}_{gameObjectName}.txt";
        destination = $"{Application.persistentDataPath}/logs/{System.DateTime.Now.ToString("yyyy-dd-MM")}";
        filePath = $"{destination}/{fileName}";

        if (!Directory.Exists(destination))
        {
            Directory.CreateDirectory(destination);
            Debug.Log(logPrefix + "Directory created!");
        }
        writer = new System.IO.StreamWriter(filePath);
    }

    public void Collect()
    {   
        var cameraPos = objectoToTrack.transform.position;
        var cameraRot = objectoToTrack.transform.rotation;
        var currentTime = System.DateTime.Now.Ticks;
        var dateTime = System.DateTime.Now;
        var dateUTC = System.DateTime.UtcNow;

        string msg = currentTime    //638173348468490800
            + ","
            + dateUTC.Ticks         //638173276468490930
            + ","
            + dateUTC.Year          //2023
            + ","
            + dateUTC.Month         //4
            + ","
            + dateUTC.Day           //17
            + ","
            + dateUTC.Hour          //11
            + ","
            + dateUTC.Minute        //27
            + ","
            + dateUTC.Second        //26
            + ","
            + dateUTC.Millisecond   //849
            + ","
            + cameraPos.x
            + ","
            + cameraPos.y
            + ","
            + cameraPos.z
            + ","
            + cameraRot.x
            + ","
            + cameraRot.y
            + ","
            + cameraRot.z
            + ","
            + cameraRot.w
            + "\n";

        Debug.Log(logPrefix + msg);
        buffer += msg;
    }

    public void Save()
    {
        Debug.Log(logPrefix + "Saving buffer...");
        writer.Write(buffer.Clone());
    }
    public void ClearBuffer()
    {
        buffer = "";
    }
}
