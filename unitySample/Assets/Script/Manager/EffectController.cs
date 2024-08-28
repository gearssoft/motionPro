using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EffectController : MonoBehaviour
{
    public EffectManager m_Main;

    public int m_Index = 0;

    public bool m_IsUse;

    public Vector3 m_Position;

    public ParticleSystem m_ParticleSyatem;

    // Start is called before the first frame update
    void Start()
    {
        //m_IsUse = false;

        if ( m_Main == null )
        {
            m_Main = FindObjectOfType<EffectManager>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        if ( m_IsUse )
        {
            if ( !m_ParticleSyatem.isPlaying )
            {
                m_IsUse = false;
                EffectReset();
            }
        }
    }

    public void EffectReset()
    {
        transform.position = m_Position;
        m_ParticleSyatem.Stop();
        m_IsUse = false;

        m_Main.m_EffectIndex1--;
    }
}
