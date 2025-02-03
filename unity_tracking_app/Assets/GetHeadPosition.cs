using System.Collections;
using UnityEngine;

public class GetHeadPosition : MonoBehaviour
{
    private PoseCollector centerEyeCollector;
    private int bufferSize = 50;
    private int bufferCount = 0;

    // Start is called before the first frame update
    void Start()
    {
        centerEyeCollector = new PoseCollector("CenterEyeAnchor");
    }

    // Update is called once per frame
    void Update()
    {
        centerEyeCollector.Collect();
        bufferCount++;
        if (bufferSize == bufferCount)
        {
            StartCoroutine(WriteToFile());
            centerEyeCollector.ClearBuffer();
            bufferCount = 0;
        }
    }
    IEnumerator WriteToFile()
    {
        centerEyeCollector.Save();
        yield return null;
    }
}
