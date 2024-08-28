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
using LZ4;

public class MrNode_Seger : MonoBehaviour
{
    private CancellationTokenSource cancellationTokenSource;
    private TcpClient tcpClient;
    private NetworkStream networkStream;

    [SerializeField] string mServerIp;
    [SerializeField] int mServerPort;

    [SerializeField] GameObject boxPrefab;
    [SerializeField] GameObject voxelPrefab;
    [SerializeField] GameObject rs2CamRoot;

    private readonly List<GameObject> boxPool = new();
    private readonly List<GameObject> VoxelPool = new();

    public void UpdateVoxels(PacketResponseSeg segDatas)
    {

        int voxelIndex = 0;

        for (int i = 0; i < segDatas.detect.Length; i++)
        {
            var detect = segDatas.detect[i];

            for (int j = 0; j < detect.point3ds.Length; j++)
            {
                var point = detect.point3ds[j];

                Vector3 __scale = new(0.05f, 0.05f, 0.05f);
                GameObject voxelGameObject;
                if (voxelIndex < VoxelPool.Count)
                {
                    voxelGameObject = VoxelPool[voxelIndex];
                    // voxelGameObject.transform.localPosition = point;
                    // voxelGameObject.transform.localScale = __scale;
                    // voxelGameObject.GetComponent<Renderer>().material.color = new Color32(detect.rgbBytes[j * 3], detect.rgbBytes[j * 3 + 1], detect.rgbBytes[j * 3 + 2], 255);
                }
                else
                {
                    voxelGameObject = Instantiate(voxelPrefab);
                    VoxelPool.Add(voxelGameObject);
                }

                voxelGameObject.SetActive(true);
                voxelGameObject.transform.SetParent(rs2CamRoot.transform, true);
                voxelGameObject.transform.localPosition = point;
                voxelGameObject.transform.localScale = __scale;
                voxelGameObject.GetComponent<Renderer>().material.color = new Color32(detect.rgbBytes[j * 3], detect.rgbBytes[j * 3 + 1], detect.rgbBytes[j * 3 + 2], 255);
                voxelIndex++;
            }
        }

        //남은 불필요한 오브젝트 비활성화
        for (int i = voxelIndex; i < VoxelPool.Count; i++)
        {
            VoxelPool[i].SetActive(false);

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
                    if (renderer.material.mainTexture == null)
                    {
                        renderer.material.mainTexture = new Texture2D(2, 2);
                    }
                    ((Texture2D)renderer.material.mainTexture).LoadImage(detect.imageBytes);

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


        // Start is called before the first frame update
        void Start()
        {
            ConnectToServer(mServerIp, mServerPort);
        }

        // Update is called once per frame
        void Update()
        {

        }


        void ConnectToServer(string ip, int port)
        {
            try
            {
                tcpClient = new TcpClient(ip, port);
                networkStream = tcpClient.GetStream();
                cancellationTokenSource = new CancellationTokenSource();
                ReceiveData(cancellationTokenSource.Token);
                Debug.Log("Connected to server");
            }
            catch (Exception ex)
            {
                Debug.LogError("Connection error: " + ex.Message + "ip: " + ip + " port: " + port);
            }
        }



        private async void ReceiveData(CancellationToken cancellationToken)
        {
            //헤더 사이즈
            byte[] buffer = new byte[8];
            int dataSize;

            while (true)
            {
                if (cancellationToken.IsCancellationRequested)
                {
                    Debug.Log("UDP Receiver cancelled");
                    return;
                }

                try
                {
                    //tcp receive
                    dataSize = await networkStream.ReadAsync(buffer, 0, buffer.Length, cancellationToken);
                    if (dataSize >= 8)
                    {
                        // Debug.Log("Received data size: " + dataSize);
                        // Assuming the header size is fixed and is the first data received
                        Header receivedHeader = PacketUtilityClass.FromByteArray<Header>(buffer);
                        int errcode = receivedHeader.Padding[0];
                        //Debug.Log("receivedHeader.MagicNumber: " + receivedHeader.MagicNumber);

                        if (PacketUtilityClass.CheckMagicNumber(receivedHeader.MagicNumber) == false)
                        {
                            Debug.LogWarning("Invalid magic number received.");
                            continue;
                        }

                        switch (receivedHeader.Command)
                        {
                            case 0x10:
                                {
                                    byte[] additionalHeader = new byte[8];
                                    dataSize = await networkStream.ReadAsync(additionalHeader, 0, additionalHeader.Length, cancellationToken);
                                    if (dataSize >= 8)
                                    {
                                        // 분석 및 처리
                                        uint timestamp = BitConverter.ToUInt32(additionalHeader, 0);
                                        uint reserved = BitConverter.ToUInt32(additionalHeader, 4);

                                        Debug.Log("received ping server time :" + timestamp);
                                    }
                                    else
                                    {
                                        Debug.LogWarning("Incomplete header received.");
                                    }

                                }
                                break; // Add this line to fix the error
                            case 0x16:
                                {
                                    byte[] additionalHeader = new byte[4];
                                    dataSize = await PacketUtilityClass.ReadFullAsync(networkStream, additionalHeader, 0, additionalHeader.Length, cancellationToken);
                                    // dataSize = await PacketUtilityClass.ReadAsyncFull(networkStream, additionalHeader, cancellationToken);

                                    if (dataSize >= 4)
                                    {
                                        uint packetLength = BitConverter.ToUInt32(additionalHeader, 0);
                                        // Debug.Log("Packet length: " + packetLength);

                                        byte[] boxDataBuffer = new byte[packetLength];
                                        //dataSize = await networkStream.ReadAsync(boxDataBuffer, 0, boxDataBuffer.Length, cancellationToken);
                                        dataSize = await PacketUtilityClass.ReadFullAsync(networkStream, boxDataBuffer, 0, boxDataBuffer.Length, cancellationToken);


                                        if (dataSize >= packetLength)
                                        {
                                            int offset = 0;
                                            List<PacketResponseSeg.SDetect> detectList = new();

                                            // 이 부분에서 intrinsics 정보를 추출합니다.
                                            uint width = BitConverter.ToUInt32(boxDataBuffer, offset);
                                            offset += 4;
                                            uint height = BitConverter.ToUInt32(boxDataBuffer, offset);
                                            offset += 4;
                                            float[] coeffs = new float[5];
                                            for (int i = 0; i < 5; i++)
                                            {
                                                coeffs[i] = BitConverter.ToSingle(boxDataBuffer, offset);
                                                offset += 4;
                                            }
                                            float fx = BitConverter.ToSingle(boxDataBuffer, offset);
                                            offset += 4;
                                            float fy = BitConverter.ToSingle(boxDataBuffer, offset);
                                            offset += 4;
                                            float ppx = BitConverter.ToSingle(boxDataBuffer, offset);
                                            offset += 4;
                                            float ppy = BitConverter.ToSingle(boxDataBuffer, offset);
                                            offset += 4;

                                            PacketResponseSeg.SIntrinsics intrinsics = new()
                                            {
                                                width = width,
                                                height = height,
                                                coeffs = coeffs,
                                                fx = fx,
                                                fy = fy,
                                                cx = ppx,
                                                cy = ppy
                                            };
                                            while (offset < packetLength)
                                            {
                                                PacketResponseSeg.SDetect detect = new();

                                                detect.x1 = BitConverter.ToSingle(boxDataBuffer, offset);
                                                detect.y1 = BitConverter.ToSingle(boxDataBuffer, offset + 4);
                                                detect.x2 = BitConverter.ToSingle(boxDataBuffer, offset + 8);
                                                detect.y2 = BitConverter.ToSingle(boxDataBuffer, offset + 12);
                                                offset += 16;

                                                detect.center_x = BitConverter.ToSingle(boxDataBuffer, offset);
                                                detect.center_y = BitConverter.ToSingle(boxDataBuffer, offset + 4);
                                                detect.depth = BitConverter.ToSingle(boxDataBuffer, offset + 8);
                                                offset += 12;

                                                uint imageSize = BitConverter.ToUInt32(boxDataBuffer, offset);
                                                offset += 4;

                                                byte[] imageBytes = new byte[imageSize];
                                                Array.Copy(boxDataBuffer, offset, imageBytes, 0, imageSize);
                                                offset += (int)imageSize;

                                                detect.imageBytes = imageBytes;

                                                // point cloud
                                                uint length = BitConverter.ToUInt32(boxDataBuffer, offset);
                                                offset += 4;

                                                // 바이트 배열에서 pointCloud 데이터를 추출합니다.
                                                byte[] pointCloudBytes = new byte[length];
                                                Array.Copy(boxDataBuffer, offset, pointCloudBytes, 0, length);
                                                offset += (int)length;

                                                int pointCloudSize = 24;
                                                int numPoints = pointCloudBytes.Length / pointCloudSize; // 각 포인트는 20바이트를 차지합니다.
                                                Vector3[] point3ds = new Vector3[numPoints];

                                                int byteOffset = 0;

                                                for (int i = 0; i < numPoints; i++)
                                                {
                                                    // Vector3 좌표 추출
                                                    float x = (float)BitConverter.ToDouble(pointCloudBytes, byteOffset);
                                                    float y = (float)BitConverter.ToDouble(pointCloudBytes, byteOffset + 8);
                                                    float z = (float)BitConverter.ToDouble(pointCloudBytes, byteOffset + 16);
                                                    point3ds[i] = new Vector3(x, y, z);

                                                    byteOffset += pointCloudSize; // 다음 포인트로 오프셋 이동
                                                }
                                                detect.point3ds = point3ds;

                                                //rgb buffer
                                                uint rgbBufferSize = BitConverter.ToUInt32(boxDataBuffer, offset);
                                                offset += 4;
                                                byte[] rgbBuffer = new byte[rgbBufferSize];
                                                Array.Copy(boxDataBuffer, offset, rgbBuffer, 0, rgbBufferSize);
                                                offset += (int)rgbBufferSize;
                                                detect.rgbBytes = rgbBuffer;

                                                detectList.Add(detect);
                                            }

                                            PacketResponseSeg boxData = new()
                                            {
                                                intrinsics = intrinsics,
                                                detect = detectList.ToArray()
                                            };

                                            UpdateBoxes(boxData);
                                            UpdateVoxels(boxData);
                                        }
                                        else
                                        {
                                            Debug.LogWarning("Incomplete box data received.");
                                        }
                                    }
                                    else
                                    {
                                        Debug.LogWarning("Incomplete header received.");
                                    }
                                }
                                break;
                            default:
                                break;
                        }
                    }

                }
                catch (OperationCanceledException)
                {
                    Debug.Log("UDP Receiver cancelled");
                    return;
                }
                catch (IOException ex)
                {
                    Debug.Log("IOException: " + ex.Message);
                    // Optionally, you could check the inner exception to see if it's a SocketException.
                    if (ex.InnerException is SocketException socketEx)
                    {
                        Debug.Log("SocketException: " + socketEx.Message);
                    }
                    return;  // Exit the method on an IOException, as the network stream is likely closed or in an invalid state.
                }
                catch (SocketException ex)
                {
                    Debug.Log("SocketException: " + ex);
                }
            }
        }

        void DisconnectFromServer()
        {
            cancellationTokenSource?.Cancel();

            networkStream?.Close();

            tcpClient?.Close();

            Debug.Log("Disconnected from server");
        }

        void OnDestroy()
        {
            DisconnectFromServer();
        }
    }
