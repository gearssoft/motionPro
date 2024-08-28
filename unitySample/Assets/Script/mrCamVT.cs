using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

using System.Linq;

public class mrCamVT : MonoBehaviour
{
    private Queue<Action> mainThreadActions;
    Thread m_receiveThread;

    UdpClient m_udpClient;

    [SerializeField] GameObject[] personObjects;

    // Start is called before the first frame update
    void Start()
    {
        m_udpClient = new UdpClient(0);

        mainThreadActions = new Queue<Action>();

        Debug.Log("start udp Thread");
        m_receiveThread = new Thread(
            new ThreadStart(ReceiveData));

        m_receiveThread.IsBackground = true;
        m_receiveThread.Start();

        PacketUtilityClass.SendPacketSetRemote(m_udpClient, "localhost", 22260);
    }

    // Update is called once per frame
    void Update()
    {
        lock (mainThreadActions)
        {
            while (mainThreadActions.Count > 0)
            {
                mainThreadActions.Dequeue()?.Invoke();
            }
        }

    }

    private void getPoseData(byte[] playload_data)
    {
    }

    void OnDestroy()
    {
        m_receiveThread.Abort();
    }


    // receive thread
    private void ReceiveData()
    {
        while (true)
        {
            try
            {

                IPEndPoint remoteIp = new(IPAddress.Any, 0);
                byte[] data = m_udpClient.Receive(ref remoteIp);

                //패킷헤더처리 
                // Header receivedHeader = Header.FromByteArray(data);
                Header receivedHeader = PacketUtilityClass.FromByteArray<Header>(data);

                if (receivedHeader.MagicNumber == PacketUtilityClass.muCheckCode)
                {
                    switch (receivedHeader.Command)
                    {
                        case 0x10:
                            Debug.Log("received ping");
                            break;
                        case 0x13:
                            {
                                PacketResponse3DPose poseData = PacketResponse3DPose.FromByteArray(data);


                                if (poseData.detect != null)
                                {
                                    lock (mainThreadActions)
                                    {
                                        mainThreadActions.Enqueue(() =>
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
                                        });
                                    }
                                }
                                else
                                    {
                                        Debug.Log("No detection data available.");
                                    }
                                }
                                break;
                            }
                    }
                else
                    {
                        Debug.Log("received unknown magic number");
                    }
                }
            catch (ThreadAbortException)
            {
                Debug.Log("ReceiveGunData thread was aborted");
                break; // exit the while loop
            }
            catch (Exception err)
            {
                print(err.ToString());
            }
        }
    }
}
