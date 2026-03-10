using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using UnityEngine.Networking;
using System.Text;

public class RobotChat : MonoBehaviour
{
    [Header("UI References")]
    public TMP_InputField chatInputField;
    public TMP_Text robotResponseText;
    public TMP_Text yourChatText;
    public Button submitButton;

    [Header("Groq Settings")]
    private string apiKey = "";
    private string apiUrl = "https://api.groq.com/openai/v1/chat/completions";
    private string model = "llama-3.3-70b-versatile";

    [System.Serializable]
    private class Config { public string groq_api_key; }

    void Start()
    {
        string configPath = Application.streamingAssetsPath + "/config.json";
        if (File.Exists(configPath))
        {
            Config cfg = JsonUtility.FromJson<Config>(File.ReadAllText(configPath));
            apiKey = cfg.groq_api_key;
        }
        else
        {
            Debug.LogError("RobotChat: config.json not found at " + configPath);
        }

        robotResponseText.gameObject.SetActive(true);
        yourChatText.gameObject.SetActive(true);
        if (submitButton != null)
            submitButton.onClick.AddListener(OnSubmit);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
        {
            OnSubmit();
        }
    }

    public void OnSubmit()
    {
        string userMessage = chatInputField.text.Trim();
        if (string.IsNullOrEmpty(userMessage)) return;

        yourChatText.text = "You: " + userMessage;
        robotResponseText.text = "Robot: thinking...";
        chatInputField.text = "";

        CheckAndExecuteCommands(userMessage);
        StartCoroutine(SendToGroq(userMessage));
    }

    private void CheckAndExecuteCommands(string userMessage)
    {
        string lower = userMessage.ToLower();
        bool isFarmCommand = lower.Contains("farm") || lower.Contains("harvest") ||
                             lower.Contains("auto") || lower.Contains("self-drive") ||
                             lower.Contains("self drive") || lower.Contains("autonomous");

        if (isFarmCommand)
        {
            if (PathMaker.Instance != null && PathMaker.Instance.roverControls != null)
            {
                PathMaker.Instance.roverControls.selfDriving.isOn = true;
                Debug.Log("[RobotChat] selfDriving toggle set to TRUE. loaded=" + PathMaker.Instance.loaded +
                    ", waypointCount=" + PathMaker.Instance.waypoints.Count);
            }
            else
            {
                Debug.LogWarning("[RobotChat] Farm command received but roverControls is null.");
            }
        }
    }

    private IEnumerator SendToGroq(string userMessage)
    {
        string jsonBody = "{" +
            "\"model\": \"" + model + "\"," +
            "\"messages\": [" +
                "{\"role\": \"system\", \"content\": \"You are a farmer robot that takes commands from a human operator. Be concise. If the user asks you to farm, harvest, auto-drive, or anything similar, confirm that you have started doing it — do not ask for clarification or more details.\"}," +
                "{\"role\": \"user\", \"content\": \"" + EscapeJson(userMessage) + "\"}" +
            "]" +
        "}";

        byte[] bodyRaw = Encoding.UTF8.GetBytes(jsonBody);

        UnityWebRequest request = new UnityWebRequest(apiUrl, "POST");
        request.uploadHandler = new UploadHandlerRaw(bodyRaw);
        request.downloadHandler = new DownloadHandlerBuffer();
        request.SetRequestHeader("Content-Type", "application/json");
        request.SetRequestHeader("Authorization", "Bearer " + apiKey);

        yield return request.SendWebRequest();

        if (request.result == UnityWebRequest.Result.Success)
        {
            string response = request.downloadHandler.text;
            string extracted = ExtractTextFromResponse(response);
            robotResponseText.text = "Robot: " + extracted;
            Debug.Log("Groq Response: " + extracted);
        }
        else
        {
            robotResponseText.text = "Robot: Error - " + request.error;
            Debug.LogError("Groq Error: " + request.error + "\n" + request.downloadHandler.text);
        }
    }

    private string ExtractTextFromResponse(string json)
    {
        const string marker = "\"content\":\"";
        int start = json.IndexOf(marker);
        if (start == -1) return "No response found.";

        start += marker.Length;
        int end = start;
        while (end < json.Length)
        {
            if (json[end] == '\\') { end += 2; continue; }  
            if (json[end] == '"') break;
            end++;
        }
        if (end >= json.Length) return "Parse error.";

        string result = json.Substring(start, end - start);
        result = result.Replace("\\n", "\n").Replace("\\\"", "\"").Replace("\\\\", "\\");
        return result;
    }

    private string EscapeJson(string s)
    {
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n").Replace("\r", "\\r");
    }
}