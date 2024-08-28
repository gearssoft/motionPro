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

using System.Threading.Tasks;

public class UiMain : MonoBehaviour
{
    [SerializeField] string m_ipAddress = "localhost";
    [SerializeField] int mnPort = 22260;

    UdpClient m_udpClient;

    [SerializeField] Button button_Ping;
    [SerializeField] Button button_req3dpoint;

    [SerializeField] Button button_StartMR;
    [SerializeField] Button button_StopMR;
    [SerializeField] Button button_Close;

    [SerializeField] GameObject m_depthCamera;

    [SerializeField] GameObject[] personObjects;

    private CancellationTokenSource cancellationTokenSource;
    // Start is called before the first frame update
    void Start()
    {
        m_udpClient = new UdpClient(0);

        cancellationTokenSource = new CancellationTokenSource();
        // await UpdatePosesPeriodically(cancellationTokenSource.Token);

        //ping 핸들러 
        button_Ping?.onClick.AddListener(async () =>
        {
            Debug.Log("ping" + m_ipAddress + ":" + mnPort);

            PacketResponsePing response = await PacketUtilityClass.SendPacketPingAsync(m_udpClient, m_ipAddress, mnPort);
            // ( PacketResponsePing response) =>


            int errcode = response.header.Padding[0];

            if (errcode == 2) // Check for timeout error code
            {
                Debug.LogError("Timed out waiting for a response");
            }
            else if (errcode == 0) //ok
            {
                Debug.Log("headerRes.MagicNumber:" + response.header.MagicNumber);
                Debug.Log("headerRes.Command:" + response.header.Command);
                Debug.Log("headerRes.TimeStamp:" + response.TimeStamp);

                // Convert timestamp to date time
                DateTime _dt = new(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                _dt = _dt.AddSeconds(response.TimeStamp).ToLocalTime();
                Debug.Log("headerRes.TimeStamp:" + _dt.ToString("yyyy-MM-dd HH:mm:ss"));
            }
            else if (errcode == 3) // 서버측 포트 가 존재하지않음
            {
                Debug.LogError("서버측 포트 가 존재하지않음");
            }
            else
            {
                Debug.LogError("알수없는 에러 " + errcode);
            }

        });

        button_req3dpoint?.onClick.AddListener(async () =>
        {
            Debug.Log("req3dpoint" + m_ipAddress + ":" + mnPort);

            var poses = await PacketUtilityClass.SendPacket3DPoseRequestAsync(m_udpClient, m_ipAddress, mnPort);
            if (poses.HasValue)
            {
                var poseData = poses.Value;
                if (poseData.detect != null)
                {
                    Debug.Log($"Number of people detected: {poseData.detect.Length}");
                    for (int i = 0; i < poseData.detect.Length; i++)
                    {
                        var detect = poseData.detect[i];
                        Debug.Log($"Person ID: {detect.personID}");
                        // Debug.Log($"");
                        // Debug.Log($"Head Position: {detect.head}");
                        // Debug.Log($"Left Hand Position: {detect.lhand}");
                        // Debug.Log($"Right Hand Position: {detect.rhand}");
                        // Debug.Log($"Left Shoulder Position: {detect.lshoulder}");
                        // Debug.Log($"Right Shoulder Position: {detect.rshoulder}");
                    }
                }
                else
                {
                    Debug.LogError("No detection data available.");
                }
            }
            else
            {
                Debug.LogError("No response or invalid response received.");
            }
        });

        button_StartMR?.onClick.AddListener(async () =>
        {
            Debug.Log("StartMR" + m_ipAddress + ":" + mnPort);
            // Start the periodic updates
            await UpdatePosesPeriodically(cancellationTokenSource.Token);
            Debug.Log("MR End");
        });

        button_StopMR?.onClick.AddListener(() =>
        {
            StopUpdatingPoses();
        });

        button_Close?.onClick.AddListener(() =>
        {
            //프로그램 종료                        
            Application.Quit();
        });

    }

    private async Task UpdatePosesPeriodically(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested)
        {
            var poses = await PacketUtilityClass.SendPacket3DPoseRequestAsync(m_udpClient, m_ipAddress, mnPort);
            if (poses.HasValue)
            {
                var poseData = poses.Value;
                if (poseData.detect != null)
                {
                    for (int i = 0; i < poseData.detect.Length; i++)
                    {
                        var detect = poseData.detect[i];
                        // Update game objects' positions here...
                        //Debug.Log($"Person {i + 1}, id : {detect.personID}");

                        // personObjects 중에서 id 가같은것이 있으며 위치를 업데이트 한다.
                        //만약 없다면 -1인것을 찾아서 아이디를 부여하고 그곳에 위치를 업데이트 한다.
                        GameObject personObject = Array.Find(personObjects, p => p.GetComponent<personDummy>().mnPersonID == detect.personID);
                        if (personObject != null)
                        {
                            // Getting the personDummy component from the found GameObject
                            personDummy _person = personObject.GetComponent<personDummy>();
                            
                            // Update the position of the existing person object
                            _person.UpdatePersonPosition(detect);
                        }
                        else
                        {
                            // Find the first person object with ID -1
                            personObject = Array.Find(personObjects, p => p.GetComponent<personDummy>().mnPersonID == -1);
                            if (personObject != null)
                            {
                                // Getting the personDummy component from the found GameObject
                                personDummy _person = personObject.GetComponent<personDummy>();

                                // Update the position of the existing person object
                                _person.UpdatePersonPosition(detect);

                                // Update the person ID
                                _person.mnPersonID = detect.personID;
                            }
                            else
                            {
                                Debug.LogError("No person object available.");
                            }
                        }
                        
                        
                    }
                }
                else
                {
                    Debug.LogError("No detection data available.");
                    await Task.Delay(1000);  // Wait for 1 second before the next request
                }
            }
            else
            {
                Debug.LogError("No response or invalid response received.");
                await Task.Delay(1000);  // Wait for 1 second before the next request
            }

            // await Task.Delay(50);  // Wait for 1 second before the next request
        }
    }

    // Call this method to stop the periodic updates
    public void StopUpdatingPoses()
    {
        cancellationTokenSource.Cancel();
    }

    public void Destroy()
    {
        StopUpdatingPoses();
        m_udpClient.Close();
    }
}
