using System.Collections;
using UnityEngine;

public class GetRightHandPosition : MonoBehaviour
{
    private PoseCollector rightHandCollector;
    private int bufferSize = 50;
    private int bufferCount = 0;


    // Start is called before the first frame update
    void Start()
    {
        rightHandCollector = new PoseCollector("RightHandAnchor");
    }

    // Update is called once per frame
    void Update()
    {
        rightHandCollector.Collect();
        bufferCount++;
        if(bufferSize == bufferCount)
        {
            StartCoroutine(WriteToFile());
            rightHandCollector.ClearBuffer();
            bufferCount = 0;
        }
    }

    IEnumerator WriteToFile()
    {
        rightHandCollector.Save();
        yield return null;
    }
}
