using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenshotHandler : MonoBehaviour
{



    private static ScreenshotHandler instance;
    private GameObject viewCameras;
    private DLLTest.RtaVideoStreamer videoStreamer = new DLLTest.RtaVideoStreamer();
    private Camera myCamera;
    private bool takeScreenshotOnNextFrame;

    private void Start()
    {
        instance = this;
        myCamera = GetComponent<Camera>();
    }


    void OnPostRender()
    {
        if (takeScreenshotOnNextFrame)
        {
            videoStreamer.StartServer(myCamera);
            RenderTexture renderTexture = myCamera.targetTexture;
            Texture2D renderResult = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.ARGB32, false);
            Rect rect = new Rect(0, 0, renderTexture.width, renderTexture.height);
            renderResult.ReadPixels(rect, 0, 0);
            byte[] byteArray = renderResult.EncodeToPNG();
            videoStreamer.SendMessage(byteArray);
        }


    }


    public static void TakeScreenshot_Static(int width, int height)
    {
        instance.TakeScreenshot(width, height);
    }

    private void TakeScreenshot(int width, int height)
    {
        myCamera.targetTexture = RenderTexture.GetTemporary(width, height, 16);
        takeScreenshotOnNextFrame = !takeScreenshotOnNextFrame;
        if (!takeScreenshotOnNextFrame)
        {
            videoStreamer.SendMessage(null);
            videoStreamer.StopServer();
        }
    }


}
