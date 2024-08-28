using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EffectManager : MonoBehaviour
{

    public GameObject m_EffectObject1;

    public List<EffectController> m_Effect1;

    public Transform m_Pooling;

    public int m_EffectMaxIndex = 50;
    public int m_EffectIndex1 = 0;

    // Start is called before the first frame update
    void Start()
    {
        this.CreateEffect();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void CreateEffect()
    {
        for ( int i = 0; i < m_EffectMaxIndex; i++ )
        {
            m_Effect1.Add( GameObject.Instantiate( m_EffectObject1, m_Pooling ).GetComponent<EffectController>() );

            m_Effect1[i].m_Main = this;
        }
    }

    public void SetEffect( Vector3 position )
    {
        if ( m_EffectIndex1 < m_Effect1.Count )
        {
            if ( !m_Effect1[m_EffectIndex1].m_IsUse )
            {
                m_Effect1[m_EffectIndex1].transform.position = position;
                m_Effect1[m_EffectIndex1].m_ParticleSyatem.Play();

                m_Effect1[m_EffectIndex1].m_IsUse = true;

                m_EffectIndex1++;
            }
        }
        else
        {
            m_Effect1.Add( GameObject.Instantiate( m_EffectObject1, m_Pooling ).GetComponent<EffectController>() );

            if ( !m_Effect1[m_EffectIndex1].m_IsUse )
            {
                m_Effect1[m_EffectIndex1].transform.position = position;
                m_Effect1[m_EffectIndex1].m_ParticleSyatem.Play();

                m_Effect1[m_EffectIndex1].m_IsUse = true;

                m_EffectIndex1++;
            }
        }
    }
}
