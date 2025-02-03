using System.Collections;
using UnityEngine;

public class GetLeftHandPosition : MonoBehaviour
{
    private PoseCollector leftHandCollector;
    private int bufferSize = 50;
    private int bufferCount = 0;


    // Start is called before the first frame update
    void Start()
    {
        leftHandCollector = new PoseCollector("LeftHandAnchor");
    }

    // Update is called once per frame
    void Update()
    {
        leftHandCollector.Collect();
        bufferCount++;
        if (bufferSize == bufferCount)
        {
            StartCoroutine(WriteToFile());
            leftHandCollector.ClearBuffer();
            bufferCount = 0;
        }
    }

    IEnumerator WriteToFile()
    {
        leftHandCollector.Save();
        yield return null;
    }
}

