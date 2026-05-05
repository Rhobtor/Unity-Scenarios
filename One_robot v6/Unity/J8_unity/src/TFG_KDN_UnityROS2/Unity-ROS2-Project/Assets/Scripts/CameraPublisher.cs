using UnityEngine;
using System;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string TopicName = "/camera/image_raw";
    public Camera camera;
    public float publishMessageFrequency = 0.5f;

    private float timeSinceLastPublish = 0f;
    private Texture2D texture2D;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(TopicName);
        texture2D = new Texture2D(GetComponent<Camera>().pixelWidth, GetComponent<Camera>().pixelHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;

        if (timeSinceLastPublish > 1f / publishMessageFrequency)
        {
            PublishCameraImage();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishCameraImage()
    {
        RenderTexture renderTexture = new RenderTexture(GetComponent<Camera>().pixelWidth, GetComponent<Camera>().pixelHeight, 24);
        GetComponent<Camera>().targetTexture = renderTexture;
        GetComponent<Camera>().Render();

        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        texture2D.Apply();
        GetComponent<Camera>().targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);

        // Voltear la imagen manualmente
        FlipImageVertically(texture2D);

        // Obtener los datos crudos de la imagen (RGB)
        byte[] imageData = texture2D.GetRawTextureData();

        ImageMsg rosImageMessage = new ImageMsg();
        rosImageMessage.data = imageData;
        rosImageMessage.width = (uint)texture2D.width;
        rosImageMessage.height = (uint)texture2D.height;
        rosImageMessage.step = (uint)(texture2D.width * 3); // RGB = 3 bytes por pixel
        rosImageMessage.encoding = "rgb8";

        ros.Publish(TopicName, rosImageMessage);
    }

    // Metodo para voltear la imagen verticalmente
    void FlipImageVertically(Texture2D texture)
    {
        var pixels = texture.GetPixels();
        for (int y = 0; y < texture.height / 2; y++)
        {
            for (int x = 0; x < texture.width; x++)
            {
                // Intercambiar las filas de p�xeles
                Color temp = pixels[y * texture.width + x];
                pixels[y * texture.width + x] = pixels[(texture.height - 1 - y) * texture.width + x];
                pixels[(texture.height - 1 - y) * texture.width + x] = temp;
            }
        }
        texture.SetPixels(pixels);
        texture.Apply();
    }
}
