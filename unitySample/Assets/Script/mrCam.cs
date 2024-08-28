using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEngine.UI;

using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

using System.Linq;

public class mrCam : MonoBehaviour
{
    UdpClient m_udpClient;
    CancellationTokenSource cancellationTokenSource;

    [SerializeField] GameObject[] personObjects;

    [SerializeField] GameObject boxPrefab;
    private List<GameObject> boxPool = new();

    // Call this method with the received box data
    public void UpdateBoxes(PacketResponseBox boxData)
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

        // Update box positions
        for (int i = 0; i < newBoxCount; i++)
        {
            var detect = boxData.detect[i];            
            boxPool[i].GetComponent<BoxPlane>().SetPosition(new Vector2(detect.x1, detect.y1), new Vector2(detect.x2, detect.y2));
        }
    }

    private void CreateNewBox()
    {
        GameObject boxGameObject = Instantiate(boxPrefab);        
        boxPool.Add(boxGameObject);
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

    // Start is called before the first frame update
    void Start()
    {
        StartUdpReceiver();
        PacketUtilityClass.SendPacketSetRemote(m_udpClient, "localhost", 22260);
    }

    private void StartUdpReceiver()
    {
        m_udpClient = new UdpClient(0);
        cancellationTokenSource = new CancellationTokenSource();
        ReceiveUdpPacket(cancellationTokenSource.Token);
    }

    // Update is called once per frame
    void Update()
    {

    }

    private async void ReceiveUdpPacket(CancellationToken cancellationToken)
    {
        while (true)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                Debug.Log("UDP Receiver cancelled");
                return;
            }

            try
            {
                UdpReceiveResult result = await m_udpClient.ReceiveAsync().WithCancellation(cancellationToken);
                byte[] data = result.Buffer;
                Header receivedHeader = PacketUtilityClass.FromByteArray<Header>(data);

                if (receivedHeader.MagicNumber != PacketUtilityClass.muCheckCode)
                {
                    Debug.Log("receivedHeader.MagicNumber != PacketUtilityClass.muCheckCode");
                    continue;
                }
                else
                {
                    switch (receivedHeader.Command)
                    {
                        case 0x10:
                            PacketResponsePing ping = PacketUtilityClass.FromByteArray<PacketResponsePing>(data);
                            Debug.Log("ping.TimeStamp: " + ping.TimeStamp);
                            break;
                        case 0x13:
                            {
                                PacketResponse3DPose poseData = PacketResponse3DPose.FromByteArray(data);

                                if (poseData.detect != null)
                                {
                                    for (int i = 0; i < poseData.detect.Length; i++)
                                    {
                                        var detect = poseData.detect[i];
                                        // Update game objects' positions here...                                        

                                        // personObjects 중에서 id 가같은것이 있으며 위치를 업데이트 한다.
                                        //만약 없다면 -1인것을 찾아서 아이디를 부여하고 그곳에 위치를 업데이트 한다.

                                        bool bFind = false;
                                        for (int j = 0; j < personObjects.Length; j++)
                                        {
                                            personDummy person = personObjects[j].GetComponent<personDummy>();
                                            if (person.mnPersonID == detect.personID)
                                            {
                                                person.UpdatePersonPosition(detect);
                                                bFind = true;
                                                break;
                                            }
                                        }

                                        if (bFind == false)
                                        {
                                            for (int j = 0; j < personObjects.Length; j++)
                                            {
                                                personDummy person = personObjects[j].GetComponent<personDummy>();
                                                if (person.mnPersonID == -1)
                                                {
                                                    person.mnPersonID = detect.personID;
                                                    person.UpdatePersonPosition(detect);
                                                    break;
                                                }
                                            }
                                        }


                                    }
                                }
                                else
                                {
                                    Debug.Log("No detection data available.");
                                }

                            }
                            break;
                        case 0x16:
                            {
                                PacketResponseBox boxData = PacketResponseBox.FromByteArray(data);
                                UpdateBoxes(boxData);

                            }
                            break;

                    }
                }

            }
            catch (OperationCanceledException)
            {
                Debug.Log("UDP Receiver cancelled");
                return;
            }
            catch (SocketException ex)
            {
                Debug.Log("SocketException: " + ex);
            }
        }
    }

    private async void OnApplicationQuit()
    {
        cancellationTokenSource.Cancel();
        await Task.Delay(1000); // Wait for 1 second

        if (m_udpClient != null)
            m_udpClient.Close();
    }
}


