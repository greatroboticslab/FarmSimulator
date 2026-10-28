using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

//Lets a trained policy drive a robot live: every recorded frame (FarmRecorder)
//goes over TCP to a policy server (farmingworld's serve_policy.py), which
//answers with the command for the coming interval.
//
//Messages, little-endian:
//  sim -> policy  "TASK" int32 n, n bytes UTF-8 JSON (task sentence, robot, crop, column names)
//                 "STEP" int32 step, int32 k, k float32 (the steps.csv row), int32 m, m bytes (RGB frame)
//                 "DONE" int32 n, n bytes UTF-8 JSON (episode results)
//  policy -> sim  after each STEP: float32 forward, float32 turn, float32 light, float32 model_ms
//
//The round trip (frame out, command back) is what lands in steps.csv as
//policy_ms; the model's own compute time is logged by the server.
public class FarmPolicyLink : IDisposable
{
	readonly string host;
	readonly int port;
	TcpClient client;
	BinaryWriter w;
	BinaryReader r;
	public bool Failed { get; private set; }
	public float LastModelMs { get; private set; }

	public FarmPolicyLink(string hostPort)
	{
		string[] hp = hostPort.Split(':');
		host = hp[0];
		port = hp.Length > 1 ? int.Parse(hp[1]) : 5555;
	}

	public bool Connect(float timeoutSeconds)
	{
		float until = Time.realtimeSinceStartup + timeoutSeconds;
		while (true)
		{
			try
			{
				client = new TcpClient();
				client.NoDelay = true;
				client.ReceiveTimeout = 10000;
				client.SendTimeout = 10000;
				client.Connect(host, port);
				NetworkStream s = client.GetStream();
				w = new BinaryWriter(s);
				r = new BinaryReader(s);
				Debug.Log("[FarmPolicyLink] connected to " + host + ":" + port);
				return true;
			}
			catch (Exception e)
			{
				if (Time.realtimeSinceStartup > until)
				{
					Debug.LogError("[FarmPolicyLink] no policy server at " + host + ":" + port + ": " + e.Message);
					Failed = true;
					return false;
				}
				System.Threading.Thread.Sleep(500);
			}
		}
	}

	void SendText(string tag, string json)
	{
		byte[] b = Encoding.UTF8.GetBytes(json);
		w.Write(Encoding.ASCII.GetBytes(tag));
		w.Write(b.Length);
		w.Write(b);
		w.Flush();
	}

	public void BeginEpisode(string json)
	{
		if (Failed) return;
		try { SendText("TASK", json); }
		catch (Exception e) { Fail(e); }
	}

	public void EndEpisode(string json)
	{
		if (Failed) return;
		try { SendText("DONE", json); }
		catch (Exception e) { Fail(e); }
	}

	//Sends one frame and its row, waits for the command. False if the link is down.
	public bool Decide(int step, float[] row, byte[] frame, out float forward, out float turn, out float light)
	{
		forward = turn = light = 0f;
		if (Failed) return false;
		try
		{
			w.Write(Encoding.ASCII.GetBytes("STEP"));
			w.Write(step);
			w.Write(row.Length);
			for (int i = 0; i < row.Length; i++) w.Write(row[i]);
			w.Write(frame.Length);
			w.Write(frame);
			w.Flush();
			forward = r.ReadSingle();
			turn = r.ReadSingle();
			light = r.ReadSingle();
			LastModelMs = r.ReadSingle();
			return true;
		}
		catch (Exception e)
		{
			Fail(e);
			return false;
		}
	}

	void Fail(Exception e)
	{
		Debug.LogError("[FarmPolicyLink] link lost, the autopilot takes over: " + e.Message);
		Failed = true;
	}

	public void Dispose()
	{
		try { if (client != null) client.Close(); } catch (Exception) { }
		client = null;
	}
}
