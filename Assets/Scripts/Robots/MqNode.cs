using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using M2MqttUnity;

public class MqNode : M2MqttUnityClient
{
	public bool enableMqtt;
	
	public void PublishInfo(float forwardInput, float turnInput, bool uvLight)
	{
		if(!enableMqtt || client == null || !client.IsConnected) return;
		
		int uLight = 0;
		if(uvLight)
			uLight = 1;
		
		
		string outString = "" + forwardInput + " " + turnInput + " " + uLight;
		client.Publish("rover", System.Text.Encoding.UTF8.GetBytes(outString), MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, false);
	}
	
    // Start is called before the first frame update
    protected override void Start()
    {
		if(!enableMqtt) autoConnect = false;
        base.Start();
    }

    // Update is called once per frame
    protected override void Update()
    {
		if(enableMqtt) base.Update();
    }
}
