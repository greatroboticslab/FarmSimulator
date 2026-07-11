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
    public CanvasGroup chatGroup;
    public TMP_InputField chatInputField;
    public TMP_Text robotResponseText;
    public TMP_Text yourChatText;
    public Button submitButton;

    [Header("Groq Settings")]
    private string apiKey = "";
    private string apiUrl = "https://api.groq.com/openai/v1/chat/completions";
    private string model = "llama-3.3-70b-versatile";
    private bool configured;

    [System.Serializable]
    private class Config { public string groq_api_key; }

    private const string systemPrompt =
        "You are a farmer robot controller. Classify the user's message and respond in exactly this two-line format:\n" +
        "ACTION: <START_FARM|STOP_FARM|NOT_A_COMMAND>\n" +
        "RESPONSE: <short message to show the user>\n\n" +
        "Use START_FARM if the user wants the robot to start autonomous farming, harvesting, or driving. " +
        "Use STOP_FARM if the user wants the robot to stop, pause, or turn off. " +
        "Use NOT_A_COMMAND if the message is unrelated to robot control — in that case set RESPONSE to 'This is not a command.' " +
        "Keep responses short and direct. Never deviate from the two-line format.";

    void Start()
    {
        string configPath = Application.streamingAssetsPath + "/config.json";
        if (File.Exists(configPath))
        {
            Config cfg = JsonUtility.FromJson<Config>(File.ReadAllText(configPath));
            apiKey = cfg != null ? cfg.groq_api_key : "";
            configured = !string.IsNullOrWhiteSpace(apiKey);
        }
        else
        {
            Debug.Log("RobotChat: optional Groq config not found at " + configPath + ". Chat commands will stay local/offline.");
        }

        if (submitButton != null)
            submitButton.onClick.AddListener(OnSubmit);

        robotResponseText?.gameObject.SetActive(true);
        yourChatText?.gameObject.SetActive(true);
        SetChatVisible(false);
    }

    public void ShowChat() => SetChatVisible(true);
    public void HideChat() => SetChatVisible(false);

    private void SetChatVisible(bool visible)
    {
        if (chatGroup == null) return;
        chatGroup.alpha = visible ? 1f : 0f;
        chatGroup.interactable = visible;
        chatGroup.blocksRaycasts = visible;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return) || Input.GetKeyDown(KeyCode.KeypadEnter))
            OnSubmit();
    }

    public void OnSubmit()
    {
        if(chatInputField == null || robotResponseText == null) return;

        string userMessage = chatInputField.text.Trim();
        if (string.IsNullOrEmpty(userMessage)) return;

        if(yourChatText != null) yourChatText.text = "You: " + userMessage;
        robotResponseText.text = "Robot: thinking...";
        chatInputField.text = "";

        if(!configured)
        {
            robotResponseText.text = "Robot: chat API is not configured.";
            return;
        }

        StartCoroutine(SendToGroq(userMessage));
    }

    private IEnumerator SendToGroq(string userMessage)
    {
        string jsonBody = "{" +
            "\"model\": \"" + model + "\"," +
            "\"messages\": [" +
                "{\"role\": \"system\", \"content\": \"" + EscapeJson(systemPrompt) + "\"}," +
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
            string raw = ExtractTextFromResponse(request.downloadHandler.text);
            string action = ParseAction(raw);
            string display = ParseResponse(raw);

            if (action == "START_FARM")
            {
                if (PathMaker.Instance != null && PathMaker.Instance.roverControls != null)
                {
                    if(PathMaker.Instance.roverControls.selfDriving != null) PathMaker.Instance.roverControls.selfDriving.isOn = true;
                    Debug.Log("[RobotChat] selfDriving set ON");
                }
            }
            else if (action == "STOP_FARM")
            {
                if (PathMaker.Instance != null && PathMaker.Instance.roverControls != null)
                {
                    if(PathMaker.Instance.roverControls.selfDriving != null) PathMaker.Instance.roverControls.selfDriving.isOn = false;
                    Debug.Log("[RobotChat] selfDriving set OFF");
                }
            }

            robotResponseText.text = "Robot: " + display;
        }
        else
        {
            robotResponseText.text = "Robot: chat service unavailable.";
            Debug.LogWarning("Groq unavailable: " + request.error);
        }
    }

    private static string ParseAction(string llmText)
    {
        foreach (string line in llmText.Split('\n'))
        {
            string trimmed = line.Trim();
            if (trimmed.StartsWith("ACTION:"))
            {
                string value = trimmed["ACTION:".Length..].Trim();
                if (value == "START_FARM" || value == "STOP_FARM" || value == "NOT_A_COMMAND")
                    return value;
            }
        }
        return "NOT_A_COMMAND";
    }

    private static string ParseResponse(string llmText)
    {
        foreach (string line in llmText.Split('\n'))
        {
            string trimmed = line.Trim();
            if (trimmed.StartsWith("RESPONSE:"))
                return trimmed["RESPONSE:".Length..].Trim();
        }
        return llmText.Trim();
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
