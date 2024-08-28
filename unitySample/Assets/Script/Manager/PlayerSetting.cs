using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[System.Serializable]
public class PlayerData
{
    public GameObject[] Data;
}

public class PlayerSetting : MonoBehaviour
{
    public EffectManager m_EffectManager;
    public MovieController m_MovieController;

    public MrScreenTcp m_CameraL;
    public MrScreenTcp m_CameraR;

    public List<PlayerData> m_PlayerData = new List<PlayerData>();
    public PlayerData m_TempData;

    private int m_LeftCount = 0;
    private int m_RightCount = 0;

    private float m_Timer = 0.0f;
    public float m_SettingTime = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        m_Timer = m_SettingTime;

        m_TempData.Data = new GameObject[17];
    }

    // Update is called once per frame
    void Update()
    {
        DataSettig();

        if ( m_PlayerData != null )
        {
            if ( m_Timer < 0 )
            {
                DataProcessing();
            }
            else
            {
                m_Timer -= Time.deltaTime;
            }

            // DataProcessing();
        }
    }

    public void DataSettig()
    {
        if( m_CameraL.kpPool != null && m_CameraR.kpPool != null )
        {

            if ( m_CameraL.kpPool.Count > 0 || m_CameraR.kpPool.Count > 0 )
            {
                if ( m_MovieController.m_Index == 0 )
                {
                    if ( !m_MovieController.m_IsStarting )
                    {
                        m_MovieController.m_IsStarting = true;
                    }
                }

                m_PlayerData.Clear();

                for ( int i = 0; i < m_CameraL.kpPool.Count; i++ )
                {
                    m_TempData.Data[m_CameraL.kpPool[i].GetComponent<KeyPointSphere>().index] = m_CameraL.kpPool[i];
                    m_LeftCount++;

                    if ( m_LeftCount == 17 )
                    {
                        m_PlayerData.Add( m_TempData );

                        // m_TempData.Data = new GameObject[17];

                        m_LeftCount = 0;
                    }
                }

                for ( int i = 0; i < m_CameraR.kpPool.Count; i++ )
                {
                    m_TempData.Data[m_CameraR.kpPool[i].GetComponent<KeyPointSphere>().index] = m_CameraR.kpPool[i];
                    m_RightCount++;

                    if ( m_RightCount == 17 )
                    {
                        m_PlayerData.Add( m_TempData );

                        // m_TempData.Data = new GameObject[17];

                        m_RightCount = 0;
                    }
                }

            }
            
            
        }
    }

    public void DataProcessing()
    {
        if ( m_PlayerData.Count > 0  )
        {
            for ( int i = 0; i < m_PlayerData.Count; i++ )
            {
                if ( m_PlayerData[i].Data[5] != null || m_PlayerData[i].Data[6] != null || m_PlayerData[i].Data[9] != null || m_PlayerData[i].Data[10] != null )
                {
                    if ( m_PlayerData[i].Data[5].transform.position.y < m_PlayerData[i].Data[9].transform.position.y )
                    {
                        // i인덱스 플레이어의 왼쪽손이 왼쪽 어깨보다 위에있을때
                        // Debug.Log( i + " 플레이어의 왼쪽손이 왼쪽 어깨보다 위에있을때" );

                        m_EffectManager.SetEffect( m_PlayerData[i].Data[9].transform.position );

                        m_Timer = m_SettingTime;
                    }

                    if ( m_PlayerData[i].Data[6].transform.position.y < m_PlayerData[i].Data[10].transform.position.y )
                    {
                        // i인덱스 플레이어의 오른쪽손이 오른쪽 어깨보다 위에있을때
                        // Debug.Log( i + " 플레이어의 오른손이 오른쪽 어깨보다 위에있을때" );

                        m_EffectManager.SetEffect( m_PlayerData[i].Data[10].transform.position );

                        m_Timer = m_SettingTime;
                    }   
                }
            }
        }
    }

}
