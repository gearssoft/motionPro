using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine.UI;

using System;
using System.IO;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

using System.Linq;

using TMPro;
using Unity.VisualScripting;

public class MrScreenTcp : MonoBehaviour
{
    private CancellationTokenSource cancellationTokenSource;
    private TcpClient tcpClient;
    private NetworkStream networkStream;

    [SerializeField] int serverPort;
    [SerializeField] string serverIp;


    [SerializeField] GameObject boxPrefab;
    [SerializeField] GameObject kpPrefab;
    [SerializeField] GameObject kptRoot;
    [SerializeField] GameObject scalePlane;

    [SerializeField] GameObject textMeshDetectCount;

    private readonly Queue<byte[]> receivedDataQueue = new Queue<byte[]>();

    private readonly List<GameObject> boxPool = new(); //segmentation

    public List<GameObject> kpPool = new(); //keypoint

    public void UpdateKeyPose(PacketResponsePose[] datas)
    {
        int totoalKpCount = 0;
        foreach (var data in datas)
        {
            if (data.detect != null)
            {
                totoalKpCount += data.detect.Length;
            }
            else
            {
                Debug.LogWarning("Invalid keypoint data received.");
                return;
            }
        }



        int currentKpCount = kpPool.Count;
        int newKpCount = totoalKpCount;

        // Adjust the box pool size
        if (newKpCount > currentKpCount)
        {
            for (int i = 0; i < newKpCount - currentKpCount; i++)
            {
                CreateNewKp();
            }
        }
        else if (newKpCount < currentKpCount)
        {
            for (int i = 0; i < currentKpCount - newKpCount; i++)
            {
                DestroyKp();
            }
        }

        int offset = 0;
        foreach (var data in datas)
        {
            //인물하나당 17개 키포인트
            /*
            0 : 코
            5 : 왼쪽어께
            6 : 오른쪽어께
            9 : 왼손
            10 : 오른손
            */

            // Update box positions and textures , data.detect.Length 는 일반적으로 17개             
            for (int i = 0; i < data.detect.Length; i++)
            {
                var detect = data.detect[i];
                var kp = kpPool[i + offset];
                // Update position
                kp.GetComponent<KeyPointSphere>().SetPosition(new Vector2(detect.x, detect.y));

                kp.GetComponent<KeyPointSphere>().index = i;
            }
            offset += data.detect.Length;
        }
    }

    private void CreateNewKp()
    {
        GameObject kpGameObject = Instantiate(kpPrefab);
        kpPool.Add(kpGameObject);
        kpGameObject.transform.SetParent(kptRoot.transform, true);

    }

    private void DestroyKp()
    {
        if (kpPool.Count > 0)
        {
            GameObject kpObject = kpPool[kpPool.Count - 1];
            kpPool.RemoveAt(kpPool.Count - 1);
            Destroy(kpObject);
        }
    }

    public void UpdateBoxes(PacketResponseSeg boxData)
    {
        int currentBoxCount = boxPool.Count;
        int newBoxCount = boxData.detect.Length;

        // Adjust the box pool size
        if (newBoxCount > currentBoxCount)
        {
            for (int i = 0; i < newBoxCount - currentBoxCount; i++)
            {
                CreateNewBox();
            }
        }
        else if (newBoxCount < currentBoxCount)
        {
            for (int i = 0; i < currentBoxCount - newBoxCount; i++)
            {
                DestroyBox();
            }
        }

        // Update box positions and textures
        for (int i = 0; i < newBoxCount; i++)
        {
            var detect = boxData.detect[i];
            var box = boxPool[i];

            // Update position
            //box.GetComponent<boxPlane>().SetPosition(new Vector2(detect.x1, detect.y1), new Vector2(detect.x2, detect.y2));
            box.GetComponent<BoxPlane>().SetPosition(new Vector2(detect.x1, detect.y2), new Vector2(detect.x2, detect.y1));

            // Update texture
            if (box.TryGetComponent<Renderer>(out var renderer))
            {
                // Destroy(renderer.material.mainTexture);
                if (renderer.material.mainTexture == null)
                {
                    renderer.material.mainTexture = new Texture2D(2, 2);
                    // detect.imageTexture.LoadImage(detect.imageBytes);
                }
                // renderer.material.mainTexture = detect.imageTexture;
                ((Texture2D)renderer.material.mainTexture).LoadImage(detect.imageBytes);

                renderer.material.shader = Shader.Find("Sprites/Default");
            }
            else
            {
                Debug.LogWarning("Renderer component not found on box object.");
            }
        }
    }

    private void CreateNewBox()
    {
        GameObject boxGameObject = Instantiate(boxPrefab);
        boxPool.Add(boxGameObject);
        boxGameObject.transform.SetParent(transform, true);

    }

    private void DestroyBox()
    {
        if (boxPool.Count > 0)
        {
            GameObject boxObject = boxPool[boxPool.Count - 1];
            boxPool.RemoveAt(boxPool.Count - 1);
            Destroy(boxObject);
        }
    }


    void DisconnectFromServer()
    {
        cancellationTokenSource?.Cancel();
    }

    void OnDestroy()
    {
        DisconnectFromServer();
    }

    void OnApplicationQuit()
    {
        Debug.Log("Application ending after " + Time.time + " seconds");
        DisconnectFromServer();
    }


    private void ReceiveData(CancellationToken cancellationToken)
    {
        try
        {
            // ConnectToServer(serverIp, serverPort);
            tcpClient = new TcpClient(serverIp, serverPort);
            networkStream = tcpClient.GetStream();

            byte[] buffer = new byte[1024*1024]; // 버퍼 크기 설정

            while (!cancellationToken.IsCancellationRequested)
            {
                int bytesRead = networkStream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 8)
                {

                    lock (receivedDataQueue)
                    {
                        receivedDataQueue.Enqueue(buffer.Take(bytesRead).ToArray());
                    }

                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("Receive error: " + ex.Message);
        }
        finally
        {
            networkStream?.Close();
            tcpClient?.Close();
        }
    }


    // Start is called before the first frame update
    void Start()
    {
        // lastPacketReceiveTick = Environment.TickCount;

        cancellationTokenSource = new CancellationTokenSource();
        Task.Run(() => ReceiveData(cancellationTokenSource.Token));


        // buttonClose.onClick.AddListener(() =>
        // {
        //     //application quit
        //     Application.Quit();
        // });

    }

    // Update is called once per frame
    void Update()
    {
        try
        {
            while (receivedDataQueue.Count > 0)
            {
                byte[] receivedData;
                lock (receivedDataQueue)
                {
                    receivedData = receivedDataQueue.Dequeue();
                }

                // 데이터 처리 로직
                ProcessReceivedData(receivedData);
            }

        }
        catch (Exception ex)
        {
            Debug.LogError("Update error: " + ex.Message);
        }
        

    }

    private void ProcessReceivedData(byte[] data)
    {
        // 8바이트 헤더 처리
        Header receivedHeader = PacketUtilityClass.FromByteArray<Header>(data.Take(8).ToArray());
        int errcode = receivedHeader.Padding[0];

        if (!PacketUtilityClass.CheckMagicNumber(receivedHeader.MagicNumber) && errcode != 0)
        {
            Debug.LogWarning("Invalid magic number received. or errcode: " + errcode);
        }
        else
        {
            switch (receivedHeader.Command)
            {
                case 0x17:
                    {
                        int detectCount = receivedHeader.Padding[1];
                        int playLoadSize = BitConverter.ToInt32(data, 8);
                        int offset = 12;

                        textMeshDetectCount.GetComponent<TextMeshPro>().text = "Detect Count: " + detectCount.ToString();

                        if (playLoadSize != data.Length - 12)
                        {
                            Debug.LogWarning("playLoadSize != data.Length - 12");
                            return;
                        }

                        // Debug.Log("detecCount: " + detacCount);
                        PacketResponsePose[] poseDatas = new PacketResponsePose[detectCount];
                        for (int i = 0; i < detectCount; i++)
                        {
                            int keypointCount = data[offset];
                            offset += 1;

                            if (keypointCount > 0)
                            {

                                // byte[] kptBuffer = new byte[8 * keypointCount]; //float 4byte * 2
                                // offset += 8 * keypointCount;

                                List<PacketResponsePose.SDetect> detectList = new();
                                for (int j = 0; j < keypointCount; j++)
                                {
                                    PacketResponsePose.SDetect detect = new()
                                    {
                                        x = BitConverter.ToSingle(data, offset + (j * 8)),
                                        y = BitConverter.ToSingle(data, offset + j * 8 + 4)
                                    };
                                    detectList.Add(detect);
                                    // offset += 8;
                                }
                                offset += 8 * keypointCount;

                                PacketResponsePose poseData = new() { detect = detectList.ToArray() };
                                poseDatas[i] = poseData;
                            }
                            else
                            {
                                //empty keypoint
                                PacketResponsePose poseData = new() { detect = new PacketResponsePose.SDetect[0] };
                                poseDatas[i] = poseData;
                                // continue;
                            }
                        }
                        UpdateKeyPose(poseDatas);

                    }
                    break;
                case 0x19:
                    {
                        if (data.Length >= 16)
                        {

                            // 데이터에서 x와 tick 값을 추출
                            float x = BitConverter.ToSingle(data, 8); // 8-11 바이트는 x 값
                            float tick = BitConverter.ToSingle(data, 12); // 12-15 바이트는 tick 값

                            // 값 확인을 위한 로그 출력
                            // Debug.Log($"Received x: {x}, tick: {tick}");

                            // textMeshFps.text = $"x: {x:F2}, tick: {1 / tick:F2}";
                            // TestCube.transform.position = new Vector3(x * 5, 0, 0);

                        }
                        break;
                    }

                default:
                    Debug.LogWarning("Unknown command received.");
                    break;
            }

        }



    }



}
