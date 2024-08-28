using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class personDummy : MonoBehaviour
{
    public int mnPersonID = -1;
    
    public GameObject m_head;
    public GameObject m_lhand;
    public GameObject m_rhand;
    public GameObject m_lshoulder;
    public GameObject m_rshoulder;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

     // 네트워크에서 수신한 데이터를 사용하여 인물의 위치를 업데이트합니다.
    public void UpdatePersonPosition(PacketResponse3DPose.SDetect detect)
    {
        // 각 게임 오브젝트의 위치를 업데이트합니다.
        // Debug.Log(detect.head.y);
        m_head.transform.localPosition  = new Vector3(detect.head.x,-detect.head.y, detect.head.z);
        m_lhand.transform.localPosition = new Vector3(detect.lhand.x,-detect.lhand.y, detect.lhand.z);
        m_rhand.transform.localPosition = new Vector3(detect.rhand.x,-detect.rhand.y, detect.rhand.z);
        m_lshoulder.transform.localPosition = new Vector3(detect.lshoulder.x,-detect.lshoulder.y, detect.lshoulder.z);
        m_rshoulder.transform.localPosition = new Vector3(detect.rshoulder.x,-detect.rshoulder.y, detect.rshoulder.z);
    }
}
