/*
  ==============================================================================

    Gradio.h
    Created: 22 Feb 2023 12:47:53pm
    Author:  Monty

  ==============================================================================
*/



#pragma once
bool generating = false;

enum TransportState
{
    Stopped,
    Starting,
    Playing,
    Stopping
};

#define callback_t std::function<void(File)>

File callGradio(String prompt, String server, File downloadsDir) {
    auto getSaveLocation = [](File downloadsDir, String prompt){
        std::cout << downloadsDir.getFullPathName() << "\n";
        std::cout << prompt << "\n";
        std::cout << downloadsDir.getFullPathName()  + "/" + prompt + ".mp4" << "\n";
        File saveLocation = File(downloadsDir.getFullPathName() + "/" + prompt + ".mp4");
        return saveLocation;
    };
    
    auto downloadAudio = [](juce::URL url, File outputFile) {
        std::unique_ptr<juce::URL::DownloadTask> res = url.downloadToFile(outputFile);

        if (res->hadError()) {
          std::cerr << "[-] Network - Failed to download model" << std::endl;
          return false;
        }
        while (!res->isFinished()){
          juce::Thread::sleep(500);
        }
        
        return true;
    };
    
    auto submitPrompt = [](juce::String prompt, String server) {
        auto callServer = [](String prompt, String server, juce::String* content) {
            juce::URL requestURL(server);
            
            juce::StringArray postData("{\"data\": [\"", prompt, "\", 2.5, 2.5, 42, 3, \"audioldm-s-full-v2\"]}");
            
            requestURL = requestURL.getChildURL("run/text2audio");
                
            requestURL = requestURL.withPOSTData(postData.joinIntoString(""));
            
            
            int statusCode = 0;
            
            juce::String extraHeader = "Accept: application/json\r\n"
            "Content-Type: application/json\r\n"
            "Sec-Fetch-Mode: cors\r\n";
            
            auto options = juce::URL::InputStreamOptions(juce::URL::ParameterHandling::inPostData)
                .withExtraHeaders(extraHeader)
                .withConnectionTimeoutMs(1000000)
                .withStatusCode(&statusCode)
                .withNumRedirectsToFollow(1);
            
            std::unique_ptr<juce::InputStream> stream = requestURL.createInputStream(options);
            
            if (stream != nullptr) {
                *content = stream->readEntireStreamAsString();
                return true;
            }
            
            if (statusCode != 0) {
                *content = "Failed to connect, status code = " + juce::String(statusCode);
                return false;
            }
            
            *content = "Failed to connect!";
            return false;
        };
        
        juce::String response;
        juce::String message;

        std::cerr << "Call server\n";
        bool submit_success = callServer(prompt, server, &response);

        message = "Done Generating\n";

        juce::var jsonObject = juce::JSON::parse(response);
        
        if (jsonObject.hasProperty("data")) {
            message = jsonObject["data"][0]["name"].toString();
        }

        return message;
    };
    
    auto getAudioURL = [](String server, juce::String childPath) {
        juce::URL requestURL(server);
        requestURL = requestURL.withNewSubPath("file=" + childPath);
        
        return requestURL;
    };
        
    generating = true;
    String fileName = submitPrompt(prompt, server);
    
    File saveLocation = getSaveLocation(downloadsDir, prompt.replaceCharacter(' ', '_'));
    
    downloadAudio(getAudioURL(server, fileName), saveLocation);
    
    return saveLocation;
}

class GradioDownloadThread : public Thread {
public:
    GradioDownloadThread (String _prompt, String _server, File _downloadsDir, callback_t _callback): Thread ("JUCE Demo Thread")
    {
        prompt = _prompt;
        server = _server;
        downloadsDir = _downloadsDir;
        callback = _callback;
    }


    void run() override
    {
        auto saveLocation = callGradio(prompt, server, downloadsDir);
        
        // because this is a background thread, we mustn't do any UI work without
        // first grabbing a MessageManagerLock..
        const MessageManagerLock mml (Thread::getCurrentThread());
        
        if (! mml.lockWasGained())  // if something is trying to kill this job, the lock
            return;                 // will fail, in which case we'd better return..
        
        callback(saveLocation);
    }

private:
    String prompt;
    String server;
    File downloadsDir;
    callback_t callback;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (GradioDownloadThread)
};
