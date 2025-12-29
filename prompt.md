on terminal
"""
Microsoft Windows [Version 10.0.19045.6466]
(c) Microsoft Corporation. All rights reserved.

D:\Physical-AI-Humanoid-Robotics-Textbook>D:/Physical-AI-Humanoid-Robotics-Textbook/app/venv/Scripts/activate.bat


(venv) D:\Physical-AI-Humanoid-Robotics-Textbook>cd app 

(venv) D:\Physical-AI-Humanoid-Robotics-Textbook\app> uvicorn main:app --reload
INFO:     Will watch for changes in these directories: ['D:\\Physical-AI-Humanoid-Robotics-Textbook\\app']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [9028] using WatchFiles
INFO:     Started server process [6900]
INFO:     Waiting for application startup.
INFO:     Application startup complete.   
INFO:     127.0.0.1:57165 - "GET / HTTP/1.1" 200 OK
INFO:     127.0.0.1:57165 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:57165 - "GET /docs HTTP/1.1" 200 OK
INFO:     127.0.0.1:57165 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:57165 - "GET /openapi.json HTTP/1.1" 200 OK
INFO:     127.0.0.1:57226 - "OPTIONS /chat/ HTTP/1.1" 200 OK
ERROR:app.skills.rag_agent:Error searching Qdrant: Error embedding content: 400 API key not valid. Please pass a 
valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
ERROR:app.skills.rag_agent:Error in process_query: Error embedding content: 400 API key not valid. Please pass a 
valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
Error in chat endpoint: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
INFO:     127.0.0.1:57226 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
WARNING:  WatchFiles detected changes in 'skills\rag_agent.py'. Reloading...
INFO:     Shutting down
INFO:     Waiting for application shutdown.
INFO:     Application shutdown complete.
INFO:     Finished server process [6900]
INFO:app.skills.rag_agent:Attempting to load GEMINI_API_KEY. Value: 'your_actual_api_key_here'
INFO:     Started server process [5008]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     127.0.0.1:57593 - "GET /docs HTTP/1.1" 200 OK
INFO:     127.0.0.1:57593 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:57593 - "GET /openapi.json HTTP/1.1" 200 OK
INFO:     127.0.0.1:57593 - "GET / HTTP/1.1" 200 OK
INFO:     127.0.0.1:57593 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
ERROR:app.skills.rag_agent:Error searching Qdrant: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
ERROR:app.skills.rag_agent:Error in process_query: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
Error in chat endpoint: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
INFO:     127.0.0.1:57650 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
ERROR:app.skills.rag_agent:Error searching Qdrant: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
ERROR:app.skills.rag_agent:Error in process_query: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
Error in chat endpoint: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
INFO:     127.0.0.1:57652 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
ERROR:app.skills.rag_agent:Error searching Qdrant: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
ERROR:app.skills.rag_agent:Error in process_query: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
Error in chat endpoint: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
INFO:     127.0.0.1:57733 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:     Shutting down
INFO:     Waiting for application shutdown.
INFO:     Application shutdown complete.
INFO:     Finished server process [5008]
INFO:     Stopping reloader process [9028]

(venv) D:\Physical-AI-Humanoid-Robotics-Textbook\app> uvicorn main:app --reload
INFO:     Will watch for changes in these directories: ['D:\\Physical-AI-Humanoid-Robotics-Textbook\\app']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [7828] using WatchFiles
INFO:app.skills.rag_agent:Attempting to load GEMINI_API_KEY. Value: 'your_actual_api_key_here'
INFO:     Started server process [4988]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     127.0.0.1:58057 - "GET /docs HTTP/1.1" 200 OK
INFO:     127.0.0.1:58057 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:58057 - "GET /openapi.json HTTP/1.1" 200 OK
INFO:     127.0.0.1:58121 - "OPTIONS /chat/ HTTP/1.1" 200 OK
ERROR:app.skills.rag_agent:Error searching Qdrant: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
ERROR:app.skills.rag_agent:Error in process_query: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
Error in chat endpoint: Error embedding content: 400 API key not valid. Please pass a valid API key. [reason: "API_KEY_INVALID"
domain: "googleapis.com"
metadata {
  key: "service"
  value: "generativelanguage.googleapis.com"
}
, locale: "en-US"
message: "API key not valid. Please pass a valid API key."
]
INFO:     127.0.0.1:58121 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:     127.0.0.1:58278 - "GET / HTTP/1.1" 200 OK
INFO:     127.0.0.1:58278 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:58278 - "GET /docs HTTP/1.1" 200 OK
INFO:     127.0.0.1:58278 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:58278 - "GET /openapi.json HTTP/1.1" 200 OK

"""

in backend ka console ye he.
"""
Tracking Prevention blocked access to storage for <URL>.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:10  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
content-script.js:22 Document already loaded, running initialization immediately
content-script.js:4 Attempting to initialize AdUnit
content-script.js:6 AdUnit initialized successfully
"""