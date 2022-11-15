using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Diagnostics;
using System;

public class myAgent : Agent
{
    public const int NUM_user = 12;
    public const int NUM_RBs = 13;
    public const int MAX_epis = 50;
    public const int NUM_BS = 4;
    public int step = 0;
    EnvironmentParameters m_ResetParams;

    public Vector2[] Users;//User Positions

    public float[] velo_Usr;
    public float[] angle_Usr;

    public float[] Ch_Gain;//user channel Gain TODO
    public float[,] Ch_Gain_OBS;
    public float Usr_Rate;// Total data rate
    public int[] Usr_Assoc;// user association
    public int[] alloc_RBs;
    public float[] eachUsr_Rate;

    public Vector2[] BS;

    public GameObject[] mBS;
    public GameObject[] mUsers;

    public override void Initialize()
    {
        Users = new Vector2[NUM_user];
        Ch_Gain = new float[NUM_user];
        Ch_Gain_OBS = new float[NUM_BS, NUM_user];
         
        Usr_Assoc = new int[NUM_user];
        alloc_RBs = new int[NUM_RBs];
        eachUsr_Rate = new float[NUM_user];
        velo_Usr = new float[NUM_user];
        angle_Usr = new float[NUM_user];
        BS = new Vector2[4];

        BS[0].x = 125.0f;
        BS[0].y = 125.0f;
        BS[1].x = 375.0f;
        BS[1].y = 125.0f;
        //BS[2].x = 375.0f;
        //BS[2].y = 125.0f;
        //BS[3].x = 375.0f;
        //BS[3].y = 375.0f;

        for (int i = 0; i < NUM_BS; ++i)
        {
            mBS[i].transform.position = new Vector3(BS[i].x, 0.0f, BS[i].y);
        }

        m_ResetParams = Academy.Instance.EnvironmentParameters;
        //SetResetParameters();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(Usr_Rate);

        for (int i = 0; i < NUM_user; ++i)
            sensor.AddObservation(eachUsr_Rate[i]);

        for (int i = 0; i < NUM_user; ++i)
            sensor.AddObservation(Usr_Assoc[i]);

        for (int i = 0; i < NUM_RBs; ++i)
            sensor.AddObservation(alloc_RBs[i]);

        for (int i = 0; i < NUM_BS; ++i)
        {
            for (int j = 0; j < NUM_user; ++j)
                sensor.AddObservation(Ch_Gain_OBS[i, j]);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float level = m_ResetParams.GetWithDefault("level", 0);

        step = this.StepCount % MAX_epis;
        float pre_rate = Usr_Rate;

        float[] pre_Assoc = new float[NUM_user];

        for (int i = 0; i < NUM_user; ++i)
            pre_Assoc[i] = Usr_Assoc[i];


        int num_changed = 0;
        //float sum_RBs = 0.0f;

        if (level != 0)
            gaussian_markov(level);
        user_move();
        user_move_go();

        //Aciton
        for (int i = 0; i < NUM_user; ++i) // ����-������ ��ҽÿ��̼�
            Usr_Assoc[i] = actionBuffers.DiscreteActions[i];

        for (int i = NUM_user; i < NUM_RBs; ++i)
        { // RB fraction Ratio
            alloc_RBs[i] = actionBuffers.DiscreteActions[i];
        }
        //UnityEngine.Debug.Log(step.ToString());
        for (int i = 0; i < NUM_user; i++) //ä�ΰ��� �ٽ� ���
            get_channel_gain(i);

        get_rate();// �ٽ� ������ ����Ʈ ���
        /*for(int i =0; i<20; i++)
        {
            UnityEngine.Debug.Log(i.ToString() + "'s Rate: " + eachUsr_Rate[i].ToString());
        }*/


        if (is_constraint_rate(60.0f))
        {
            SetReward(-1000.0f);
            //UnityEngine.Debug.Log("Each users DataRate Threshold constraint...");
            EndEpisode();
        }
        if (pre_rate > Usr_Rate)
        {  // compare data rate & previous rate
            AddReward(Usr_Rate * 0.2f);
        }
        else
            AddReward(Usr_Rate + Usr_Rate - pre_rate);

        num_changed = is_changed(pre_Assoc);
        if (num_changed != 0)
        {// Check handover
            for (int i = 0; i < num_changed; ++i)
                AddReward(-150.0f);
        }
        //AddReward(step * 5.0f);
        //step = step + 1;
    }

    public override void OnEpisodeBegin()
    {
        SetResetParameters();
    }

    public void SetResetParameters()
    {
        step = 0; // Reset episode count
        for (int i = 0; i < NUM_user; ++i)
        {
            Users[i].x = UnityEngine.Random.Range(0.0f, 500.0f);
            Users[i].y = UnityEngine.Random.Range(0.0f, 500.0f);
        }

        for (int i = 0; i < NUM_user; ++i)
        {
            velo_Usr[i] = 1.0f;
            angle_Usr[i] = UnityEngine.Random.Range(0.0f, 2 * UnityEngine.Random.Range(0, 2 * Mathf.PI));
        }
        for (int i = 0; i < NUM_user; i++)
            Usr_Assoc[i] = (int)Mathf.Floor(UnityEngine.Random.Range(0.0f, 3.9f));

        for (int i = 0; i < NUM_RBs; i++)
            alloc_RBs[i] = (int)Mathf.Floor(UnityEngine.Random.Range(0.0f, 19.0f));

        for (int i = 0; i < NUM_user; i++)
            get_channel_gain(i);

        get_rate();

    }

    public int is_changed(float[] pre)
    {
        int i = 0;
        for (int j = 0; j < NUM_user; ++j)
        {
            if (Usr_Assoc[i] != pre[i])
                i = i + 1;
        }
        return i;
    }

    public void user_move()
    {
        for (int i = 0; i < NUM_user; ++i)
        {
            Users[i].x = Users[i].x + velo_Usr[i] * Mathf.Cos(angle_Usr[i]);
            Users[i].y = Users[i].y + velo_Usr[i] * Mathf.Sin(angle_Usr[i]);
        }
    }
    public void user_move_go()
    {
        for(int i = 0; i < NUM_user; ++i)
        {
            mUsers[i].transform.position = new Vector3(Users[i].x, 0.0f, Users[i].y);
        }
    }
    public void gaussian_markov(float lev)
    {
        float velo_mean = lev;

        //TODO->���̵�
        float angle_mean = 0.0f;
        for (int j = 0; j < NUM_user; j++)
        {
            float alpha1 = UnityEngine.Random.Range(0.0f, 1.0f);
            float alpha2 = 1 - alpha1;
            float alpha3 = Mathf.Sqrt(1 - alpha1 * alpha1);

            velo_Usr[j] = (alpha1 * velo_Usr[j] +
                           alpha2 * velo_mean +
                           alpha3 * UnityEngine.Random.Range(0.0f, 1.0f)
                           );
            angle_Usr[j] = (alpha1 * angle_Usr[j] +
                           alpha2 * angle_mean +
                           alpha3 * UnityEngine.Random.Range(0.0f, 1.0f)
                           );
        }
    }

    public bool is_constraint_alloc(int RB)
    {
        if (RB > NUM_RBs)
            return false;
        return true;
    }
    public bool is_constraint_rate(float Threshold)
    {
        for (int i = 0; i < NUM_user; ++i)
        {
            if (eachUsr_Rate[i] < Threshold)
            {
                return true;
            }
        }
        return false;
    }

    public float randn(float mean, float stddev)
    {
        float r1 = UnityEngine.Random.Range(0.0f, 1.0f);
        float r2 = UnityEngine.Random.Range(0.0f, 1.0f);

        float rand = Mathf.Sqrt(-2.0f * Mathf.Log(r1)) * Mathf.Sin(2.0f * Mathf.PI * r2);
        return rand;
        //return mean + stddev * rand;
    }

    public void get_channel_gain(int idx)
    {
        float eta = 0.7f; // path loss exponent
        int my_bs = Usr_Assoc[idx]; // BS
        float dist;
        float g;
        for (int i = 0; i < 4; ++i)
        {
            dist = Vector2.Distance(new Vector2(Users[idx].x, Users[idx].y), new Vector3(BS[i].x, BS[i].y));
            g = 10.0f / Mathf.Pow(dist, eta);
            //UnityEngine.Debug.Log(my_bs.ToString() + "    " + idx.ToString());
            Ch_Gain_OBS[i, idx] = g;
            if (my_bs == i)
                Ch_Gain[idx] = g;
        }

    }


    public void get_rate()
    {
        float rate = 0;
        int num_RBs;
        for (int i = 0; i < NUM_user; ++i)
        {
            num_RBs = 1;
            for (int j = 0; j < NUM_RBs; ++j)
            {
                if (alloc_RBs[j] == i)
                    num_RBs = num_RBs + 1;
            }
            int BW = 180000 * num_RBs;
            //float n_dbm = -174 + 10 * Mathf.Log10(BW);
            //float n_linear = Mathf.Pow(10, -3) * (Mathf.Pow(10, (n_dbm / 10)));
            float pt = 4.0f / 10;
            //Debug.Log("gain from array: " + Ch_Gain[step, i]);

            float snr = (pt * Ch_Gain[i]) / 174f;
            //float snr = pt * Ch_Gain[this.StepCount, i] / n_linear;

            eachUsr_Rate[i] = BW * Mathf.Log(1 + snr);
            //UnityEngine.Debug.Log(eachUsr_Rate[19].ToString());

            rate = rate + eachUsr_Rate[i];

            //Debug.Log("rate" + i.ToString() + ":" + rate.ToString());
        }
        Usr_Rate = rate;
    }
}
