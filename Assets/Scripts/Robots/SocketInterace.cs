using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System;

public class SocketInterace : MonoBehaviour
{
	
	public static SocketInterace Instance;
	
	public string ip = "localhost";
	public bool started;
	public bool enableSocket;
	public DebugRover rover;
	public RenderTexture frame;
	public Material testFrame;
	public int bufferSize = 8192;//8192;
	public int headerSize = 5;
	
	public Socket client;
	
    // Start is called before the first frame update
    void Start()
    {
		
		Instance = this;
		
    }
	
	public void Connect() {
		if(!enableSocket) return;
		
		IPHostEntry host = Dns.GetHostEntry(ip);
		IPAddress ipAddress = host.AddressList.Length > 0 ? host.AddressList[0] : IPAddress.Loopback;
		
		IPEndPoint remoteEP = new IPEndPoint(ipAddress, 9001);
		
        client = new Socket(ipAddress.AddressFamily,
                SocketType.Stream, ProtocolType.Tcp);
				
		client.SendBufferSize = bufferSize;
		
		try {
			client.Connect(remoteEP);
			Debug.Log("Socket connected to " + client.RemoteEndPoint.ToString());
			started = true;
		}
		catch(SocketException e) {
			Debug.LogWarning("SocketInterface: optional socket connection failed: " + e.Message);
			started = false;
		}
		
	}
	
	private float frm = 0;
	private Texture2D tex;

    // Update is called once per frame
    void Update()
    {
		if(!enableSocket || !started || rover == null || frame == null || client == null || !client.Connected) return;
		
		int dataSize = bufferSize - headerSize;
		
		int chunkId = 0;
        
		if(frm < 5 && rover != null) {
			frm += Time.deltaTime;
			return;
		}
		
		int res = 512;
		int imgSize = res*res*3;
		
		RenderTexture.active = frame;
		if(tex == null) tex = new Texture2D(res, res, TextureFormat.RGB24, false);
		tex.ReadPixels(new Rect(0, 0, res, res), 0, 0);
		tex.Apply();
		
		
		
		if(started) {
			
			rover.netControlled = true;
			
			string output = "";
			output += rover.latitude;
			output += " ";
			output += rover.longitude;
			output += " ";
			output += rover.heading;
			output += " ";
			output += rover.vel;
			output += " ";
			//output += rover.currentWaypoint.x;
			output += rover.currentWaypointLATLNG.x;
			output += " ";
			//output += rover.currentWaypoint.y;
			output += rover.currentWaypointLATLNG.y;
			output += " ";
			output += imgSize;
			output += "\n";
			
			int msgSize = output.Length;
			byte[] intBytes = BitConverter.GetBytes(msgSize);
			output = "" + (char)0 + intBytes[0] + intBytes[1] + intBytes[2] + intBytes[3] + output;
			
			byte[] messageSent = Encoding.ASCII.GetBytes(output);
			int byteSent = client.Send(messageSent);
			//Debug.Log("PosData: " + output);
			
			byte[] messageRecv = new byte[bufferSize];
			client.Receive(messageRecv, 0, bufferSize, SocketFlags.None);
			var response = Encoding.UTF8.GetString(messageRecv);
			//Debug.Log("Got: " + response);
			
			
			//testFrame.SetTexture("_MainTex", tex);
			
			//GetPixels32 returns the same bottom-left row-major order the per-pixel loop walked
			Color32[] pixels = tex.GetPixels32();
			byte[] imgBytes = new byte[imgSize];
			int bPos = 0;
			for(int i = 0; i < pixels.Length && bPos + 2 < imgBytes.Length; i++) {
				imgBytes[bPos++] = pixels[i].r;
				imgBytes[bPos++] = pixels[i].g;
				imgBytes[bPos++] = pixels[i].b;
			}
			
			byteSent = client.Send(imgBytes);
			messageRecv = new byte[bufferSize];
			int recvCount = client.Receive(messageRecv, 0, bufferSize, SocketFlags.None);
			response = Encoding.UTF8.GetString(messageRecv, 0, recvCount);
			Debug.Log("Got: " + response);

			string[] dats = response.Split(' ');

			if(dats.Length >= 3 &&
			   float.TryParse(dats[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float forward) &&
			   float.TryParse(dats[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float turn) &&
			   float.TryParse(dats[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float lightInp)) {
				rover.netForwardInput = forward;
				rover.netTurnInput = turn;
				rover.netLightInput = lightInp;
			}
			else {
				Debug.LogWarning("SocketInterface: unexpected control response, ignoring: " + response);
			}
			
			
		}
		
    }
}
