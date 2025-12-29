mere vscode me
"""
[{
	"resource": "/D:/Physical-AI-Humanoid-Robotics-Textbook/app/skills/rag_agent.py",
	"owner": "Pylance",
	"code": {
		"value": "reportUndefinedVariable",
		"target": {
			"$mid": 1,
			"path": "/microsoft/pylance-release/blob/main/docs/diagnostics/reportUndefinedVariable.md",
			"scheme": "https",
			"authority": "github.com"
		}
	},
	"severity": 4,
	"message": "\"qdrant_url\" is not defined",
	"source": "Pylance",
	"startLineNumber": 40,
	"startColumn": 27,
	"endLineNumber": 40,
	"endColumn": 37,
	"origin": "extHost1"
}]
"qdrant_url" is not defined
"""

error in terminal
"""
Microsoft Windows [Version 10.0.19045.6466]
(c) Microsoft Corporation. All rights reserved.

D:\Physical-AI-Humanoid-Robotics-Textbook>D:/Physical-AI-Humanoid-Robotics-Textbook/app/venv/Scripts/activate.bat


(venv) D:\Physical-AI-Humanoid-Robotics-Textbook>cd app 

(venv) D:\Physical-AI-Humanoid-Robotics-Textbook\app>uvicorn main:app --reload
INFO:     Will watch for changes in these directories: ['D:\\Physical-AI-Humanoid-Robotics-Textbook\\app']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12204] using WatchFiles
INFO:     Started server process [7872]
INFO:     Waiting for application startup.
INFO:     Application startup complete.   
INFO:     127.0.0.1:52216 - "OPTIONS /chat/ HTTP/1.1" 200 OK
INFO:app.skills.rag_agent:Initializing RAGAgent singleton...
--- [RAG AGENT FORENSIC DEBUG] ---
  - Key Loaded: Yes
  - repr(key): 'your_actual_api_key_here'
  - Exact length: 24
  - First 10 chars: your_actua
  - Last 5 chars: _here
--- [END FORENSIC DEBUG] ---
ERROR:app.skills.rag_agent:Error in process_query: name 'qdrant_url' is not defined
Error in chat endpoint: name 'qdrant_url' is not defined
INFO:     127.0.0.1:52216 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:app.skills.rag_agent:Initializing RAGAgent singleton...
--- [RAG AGENT FORENSIC DEBUG] ---
  - Key Loaded: Yes
  - repr(key): 'your_actual_api_key_here'
  - Exact length: 24
  - First 10 chars: your_actua
  - Last 5 chars: _here
--- [END FORENSIC DEBUG] ---
ERROR:app.skills.rag_agent:Error in process_query: name 'qdrant_url' is not defined
Error in chat endpoint: name 'qdrant_url' is not defined
INFO:     127.0.0.1:52218 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:app.skills.rag_agent:Initializing RAGAgent singleton...
--- [RAG AGENT FORENSIC DEBUG] ---
  - Key Loaded: Yes
  - repr(key): 'your_actual_api_key_here'
  - Exact length: 24
  - First 10 chars: your_actua
  - Last 5 chars: _here
--- [END FORENSIC DEBUG] ---
ERROR:app.skills.rag_agent:Error in process_query: name 'qdrant_url' is not defined
Error in chat endpoint: name 'qdrant_url' is not defined
INFO:     127.0.0.1:52219 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:app.skills.rag_agent:Initializing RAGAgent singleton...
--- [RAG AGENT FORENSIC DEBUG] ---
  - Key Loaded: Yes
  - repr(key): 'your_actual_api_key_here'
  - Exact length: 24
  - First 10 chars: your_actua
  - Last 5 chars: _here
--- [END FORENSIC DEBUG] ---
ERROR:app.skills.rag_agent:Error in process_query: name 'qdrant_url' is not defined
Error in chat endpoint: name 'qdrant_url' is not defined
INFO:     127.0.0.1:52250 - "POST /chat/ HTTP/1.1" 500 Internal Server Error
INFO:     127.0.0.1:52266 - "GET /docs HTTP/1.1" 200 OK
INFO:     127.0.0.1:52266 - "GET /.well-known/appspecific/com.chrome.devtools.json HTTP/1.1" 404 Not Found
INFO:     127.0.0.1:52266 - "GET /openapi.json HTTP/1.1" 200 OK


"""

error in console
"""
8Tracking Prevention blocked access to storage for <URL>.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui.css.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
docs:1  Tracking Prevention blocked access to storage for https://cdn.jsdelivr.net/npm/swagger-ui-dist@5/swagger-ui-bundle.js.
content-script.js:22 Document already loaded, running initialization immediately
content-script.js:4 Attempting to initialize AdUnit
content-script.js:6 AdUnit initialized successfully
"""