using System.Collections;
using System.Collections.Generic;
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
    private string apiKey = "YOUR_API_KEY_KEY";
    private string apiUrl = "https://api.groq.com/openai/v1/chat/completions";
  private string model = "llama-3.3-70b-versatile";

    void Start()
    {
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

        StartCoroutine(SendToGroq(userMessage));
    }

    private IEnumerator SendToGroq(string userMessage)
    {
        string jsonBody = "{" +
            "\"model\": \"" + model + "\"," +
            "\"messages\": [" +
                "{\"role\": \"system\", \"content\": \"You are a farmer robot taking requests from a human. Do not give long responses. Be concise and practical.\"}," +
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
        int end = json.IndexOf("\"", start);
        if (end == -1) return "Parse error.";

        string result = json.Substring(start, end - start);
        result = result.Replace("\\n", "\n").Replace("\\\"", "\"").Replace("\\\\", "\\");
        return result;
    }

    private string EscapeJson(string s)
    {
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n").Replace("\r", "\\r");
    }
}