using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Threading.Tasks;

using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
// using UnityEditor.Experimental.GraphView;
// using UnityEditor.PackageManager;

using System.IO;
using System.IO.Compression;


using System.Linq;
using System.Runtime.InteropServices;
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Header
{
    public uint MagicNumber;
    public byte Command;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
    public byte[] Padding;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketResponsePing
{
    public Header header;
    public uint TimeStamp;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketResponseSeg
{
    public struct SDetect
    {
        public float x1, y1, x2, y2;
        public float center_x, center_y, depth;        
        // public Texture2D imageTexture;
        public byte[] imageBytes;

        public Vector3[] point3ds;
        public byte[] rgbBytes;     
    }

    public struct SIntrinsics
    {
        public float fx, fy, cx, cy;
        public float[] coeffs;
        public float width, height;
    }

    public SDetect[] detect;
    public SIntrinsics intrinsics;
}


[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketResponsePose
{
    public struct SDetect
    {
        public float x, y;
    }

    public SDetect[] detect;
}


[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketResponseBox
{
    public struct SDetect
    {
        public float x1;
        public float y1;
        public float x2;
        public float y2;

        public float depth;

    }

    public Header header;
    public SDetect[] detect;

    public static PacketResponseBox FromByteArray(byte[] byteArray)
    {
        //header 크기만큼 잘라내서 확인
        Header resHeader = PacketUtilityClass.FromByteArray<Header>(byteArray.Take(8).ToArray());
        int detectSize = resHeader.Padding[1];

        // Remaining bytes after the header
        byte[] remainingBytes = byteArray.Skip(8).ToArray();

        PacketResponseBox packetResponseBox = new();
        packetResponseBox.header = resHeader;
        packetResponseBox.detect = new PacketResponseBox.SDetect[detectSize];

        //PacketResponseBox.SDetect 의 크기 구하기 
        int _DataSize = Marshal.SizeOf(typeof(PacketResponseBox.SDetect));

        for (int i = 0; i < detectSize; i++)
        {
            byte[] detectBytes = remainingBytes.Skip(i * _DataSize).Take(_DataSize).ToArray();

            GCHandle handle = GCHandle.Alloc(detectBytes, GCHandleType.Pinned);
            PacketResponseBox.SDetect detect = (PacketResponseBox.SDetect)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(PacketResponseBox.SDetect));
            handle.Free();

            packetResponseBox.detect[i] = detect;
        }

        return packetResponseBox;
    }
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketResponse3DPose
{
    public struct SDetect
    {
        public int personID;
        public Vector3 head;
        public Vector3 lhand;
        public Vector3 rhand;
        public Vector3 lshoulder;
        public Vector3 rshoulder;
    }

    public Header header;
    public SDetect[] detect;

    public static PacketResponse3DPose FromByteArray(byte[] byteArray)
    {
        //header 크기만큼 잘라내서 확인
        Header resHeader = PacketUtilityClass.FromByteArray<Header>(byteArray.Take(8).ToArray());
        int detectSize = resHeader.Padding[1];

        // Remaining bytes after the header
        byte[] remainingBytes = byteArray.Skip(8).ToArray();

        PacketResponse3DPose packetResponse3DPose = new();
        packetResponse3DPose.header = resHeader;
        packetResponse3DPose.detect = new PacketResponse3DPose.SDetect[detectSize];

        //PacketResponse3DPose.SDetect 의 크기 구하기 
        int _DataSize = Marshal.SizeOf(typeof(PacketResponse3DPose.SDetect));

        for (int i = 0; i < detectSize; i++)
        {
            byte[] detectBytes = remainingBytes.Skip(i * _DataSize).Take(_DataSize).ToArray();

            GCHandle handle = GCHandle.Alloc(detectBytes, GCHandleType.Pinned);
            PacketResponse3DPose.SDetect detect = (PacketResponse3DPose.SDetect)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(PacketResponse3DPose.SDetect));
            handle.Free();

            packetResponse3DPose.detect[i] = detect;
        }

        return packetResponse3DPose;


    }
}

public static class PacketUtilityClass
{
    public static uint muCheckCode = 20231026;

    public static Boolean CheckMagicNumber(uint magicNumber)
    {
        if (magicNumber == muCheckCode)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static byte[] ToByteArray<T>(T structure) where T : struct
    {
        int size = Marshal.SizeOf(typeof(T));
        byte[] arr = new byte[size];
        IntPtr ptr = Marshal.AllocHGlobal(size);

        Marshal.StructureToPtr(structure, ptr, true);
        Marshal.Copy(ptr, arr, 0, size);
        Marshal.FreeHGlobal(ptr);

        return arr;
    }

    //바이트 배열을 해당형식의 구조체로 변환하는 Generic 메서드
    //사용예 > PacketResponsePing packet = FromByteArray<PacketResponsePing>(byteArray);
    public static T FromByteArray<T>(byte[] byteArray) where T : struct
    {
        GCHandle handle = GCHandle.Alloc(byteArray, GCHandleType.Pinned);
        T packet = (T)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(T));
        handle.Free();
        return packet;
    }

    //빈구조체를 생성하는 Generic 메서드
    //사용예> PacketResponsePing packet = FromVoid<PacketResponsePing>();
    public static T FromVoid<T>() where T : new()
    {
        T structure = new();
        if (typeof(T) == typeof(PacketResponsePing))
        {
            PacketResponsePing temp = (PacketResponsePing)Convert.ChangeType(structure, typeof(PacketResponsePing));
            temp.header = new Header
            {
                Padding = new byte[3]
            };
            structure = (T)Convert.ChangeType(temp, typeof(T));
        }

        return structure;
    }
    public static async Task<int> ReadFullAsync(NetworkStream stream, byte[] buffer, int offset, int size, CancellationToken cancellationToken)
    {
        int totalRead = 0;
        while (totalRead < size)
        {
            int bytesRead = await stream.ReadAsync(buffer, offset + totalRead, size - totalRead, cancellationToken);
            if (bytesRead == 0)
                throw new IOException("Connection closed by remote host.");
            totalRead += bytesRead;
        }
        return totalRead;
    }

    public static async Task<UdpReceiveResult> ReceiveResponseAsync(UdpClient client, TimeSpan timeout)
    {
        using var cts = new CancellationTokenSource(timeout);
        try
        {
            var receiveTask = client.ReceiveAsync();
            var completedTask = await Task.WhenAny(receiveTask, Task.Delay(timeout));
            if (completedTask == receiveTask)
            {
                return await receiveTask;  // Return the result of ReceiveAsync
            }
            else
            {
                throw new TimeoutException();  // Task.Delay completed first because of a timeout
            }
        }
        catch (Exception ex)
        {
            Debug.Log(ex.ToString());
            // Optionally log the exception (e.g., Debug.LogException(ex))
            throw;  // Re-throw the exception
        }
    }

    public static void SendPacketPing(UdpClient udpClient, string ip, int port)
    {
        Debug.Log("sendPacketPing");
        try
        {
            Header header = new();
            header.MagicNumber = muCheckCode;
            header.Command = 0x10;

            // Header 객체의 바이트 배열 표현을 headerBytes 변수에 할당합니다
            byte[] headerBytes = ToByteArray(header);

            //UdpClient 클래스의 Send 메서드를 호출하여 headerBytes 배열을 지정된 IP 주소와 포트로 UDP 패킷으로 전송합니다.
            udpClient.Send(headerBytes, headerBytes.Length, ip, port);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
        }
    }
    public static void SendPacketSetRemote(UdpClient udpClient, string ip, int port)
    {
        Debug.Log("sendPacketPing");
        try
        {
            Header header = new();
            header.MagicNumber = muCheckCode;
            header.Command = 0x14;

            // Header 객체의 바이트 배열 표현을 headerBytes 변수에 할당합니다
            byte[] headerBytes = ToByteArray(header);

            //UdpClient 클래스의 Send 메서드를 호출하여 headerBytes 배열을 지정된 IP 주소와 포트로 UDP 패킷으로 전송합니다.
            udpClient.Send(headerBytes, headerBytes.Length, ip, port);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
        }
    }
    public static void SendPacketResetRemote(UdpClient udpClient, string ip, int port)
    {
        Debug.Log("sendPacketPing");
        try
        {
            Header header = new();
            header.MagicNumber = muCheckCode;
            header.Command = 0x15;

            // Header 객체의 바이트 배열 표현을 headerBytes 변수에 할당합니다
            byte[] headerBytes = ToByteArray(header);

            //UdpClient 클래스의 Send 메서드를 호출하여 headerBytes 배열을 지정된 IP 주소와 포트로 UDP 패킷으로 전송합니다.
            udpClient.Send(headerBytes, headerBytes.Length, ip, port);
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
        }
    }

    public static async Task SendPacketSetRemoteAsync(TcpClient tcpClient)
    {
        Debug.Log("sendPacketSetRemoteAsync");
        try
        {
            if (tcpClient != null && tcpClient.Connected)
            {
                Header header = new()
                {
                    MagicNumber = muCheckCode,
                    Command = 0x14
                };

                byte[] headerBytes = ToByteArray(header);

                NetworkStream networkStream = tcpClient.GetStream();
                await networkStream.WriteAsync(headerBytes, 0, headerBytes.Length);
            }
            else
            {
                Debug.LogWarning("TCP client is not connected.");
            }
        }
        catch (Exception e)
        {
            Debug.LogError(e.ToString());
        }

    }

    public static async Task SendPacketPingAsync(TcpClient tcpClient)
    {
        Debug.Log("send ping");
        try
        {
            if (tcpClient != null && tcpClient.Connected)
            {
                Header header = new();
                header.MagicNumber = muCheckCode;
                header.Command = 0x10;

                byte[] headerBytes = ToByteArray(header);

                NetworkStream networkStream = tcpClient.GetStream();
                await networkStream.WriteAsync(headerBytes, 0, headerBytes.Length);
            }
            else
            {
                Debug.LogWarning("TCP client is not connected.");
            }
        }
        catch (Exception e)
        {
            Debug.LogError(e.ToString());
        }
    }

    public static async Task<PacketResponsePing> SendPacketPingAsync(UdpClient udpClient, string ip, int port)
    {
        Debug.Log("sendPacketPing");
        try
        {
            Header header = new();
            header.MagicNumber = muCheckCode;
            header.Command = 0x10;

            // Header 객체의 바이트 배열 표현을 headerBytes 변수에 할당합니다
            byte[] headerBytes = ToByteArray(header);

            //UdpClient 클래스의 Send 메서드를 호출하여 headerBytes 배열을 지정된 IP 주소와 포트로 UDP 패킷으로 전송합니다.
            udpClient.Send(headerBytes, headerBytes.Length, ip, port);

            // Wait for a response
            var response = await ReceiveResponseAsync(udpClient, TimeSpan.FromSeconds(1));

            //패킷을 받았을때 처리
            PacketResponsePing _res = FromByteArray<PacketResponsePing>(response.Buffer);
            return _res;
        }
        catch (TimeoutException)
        {
            Debug.LogError("Timed out waiting for a response");
            PacketResponsePing _res = FromVoid<PacketResponsePing>();
            _res.header.Padding[0] = 2; //timeout errocode
            return _res;
        }
        catch (Exception e)
        {
            PacketResponsePing _res = FromVoid<PacketResponsePing>();
            _res.header.Padding[0] = 3; // 서버측 포트 가 존재하지않음
            Debug.Log(e.ToString());
            return _res;
        }
    }


    public static async Task<Nullable<PacketResponse3DPose>> SendPacket3DPoseRequestAsync(UdpClient udpClient, string ip, int port)
    {
        try
        {
            Header header = new();
            header.MagicNumber = muCheckCode;
            header.Command = 0x13;

            byte[] headerBytes = ToByteArray(header);
            udpClient.Send(headerBytes, headerBytes.Length, ip, port);

            var response = await ReceiveResponseAsync(udpClient, TimeSpan.FromSeconds(1));

            //header 크기만큼 잘라내서 확인
            Header resHeader = FromByteArray<Header>(response.Buffer.Take(8).ToArray());
            int detectSize = resHeader.Padding[1];

            // Remaining bytes after the header
            byte[] remainingBytes = response.Buffer.Skip(8).ToArray();

            PacketResponse3DPose packetResponse3DPose = new();
            packetResponse3DPose.header = resHeader;
            packetResponse3DPose.detect = new PacketResponse3DPose.SDetect[detectSize];

            //PacketResponse3DPose.SDetect 의 크기 구하기 
            int _DataSize = Marshal.SizeOf(typeof(PacketResponse3DPose.SDetect));

            for (int i = 0; i < detectSize; i++)
            {
                byte[] detectBytes = remainingBytes.Skip(i * _DataSize).Take(_DataSize).ToArray();

                GCHandle handle = GCHandle.Alloc(detectBytes, GCHandleType.Pinned);
                PacketResponse3DPose.SDetect detect = (PacketResponse3DPose.SDetect)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof(PacketResponse3DPose.SDetect));
                handle.Free();

                packetResponse3DPose.detect[i] = detect;
            }

            return packetResponse3DPose;
        }
        catch (TimeoutException)
        {
            Debug.LogError("Timed out waiting for a response");
            return default;
        }
        catch (Exception e)
        {
            Debug.Log(e.ToString());
            return default;
        }
    }

}


public static class TaskExtensions
{
    public static async Task<T> WithCancellation<T>(this Task<T> task, CancellationToken cancellationToken)
    {
        var tcs = new TaskCompletionSource<bool>();
        using (cancellationToken.Register(s => ((TaskCompletionSource<bool>)s).TrySetResult(true), tcs))
            if (task != await Task.WhenAny(task, tcs.Task))
                throw new OperationCanceledException(cancellationToken);
        return await task;
    }
}
