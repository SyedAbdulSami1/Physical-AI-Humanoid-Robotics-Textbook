
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook (main)
$ source D:/Physical-AI-Humanoid-Robotics-Textbook/.venv/Scripts/activate
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook (main)
$    2 pip uninstall psycopg2-binary psycopg2 -y
bash: 2: command not found
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook (main)
$ pip uninstall psycopg2-binary psycopg2 -y
Found existing installation: psycopg2-binary 2.9.11
Uninstalling psycopg2-binary-2.9.11:
  Successfully uninstalled psycopg2-binary-2.9.11
WARNING: Skipping psycopg2 as it is not installed.
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook (main)
$ pip install asyncpg==0.29.0
Collecting asyncpg==0.29.0
  Using cached asyncpg-0.29.0-cp312-cp312-win_amd64.whl.metadata (4.5 kB)
Using cached asyncpg-0.29.0-cp312-cp312-win_amd64.whl (530 kB)
Installing collected packages: asyncpg
  Attempting uninstall: asyncpg
    Found existing installation: asyncpg 0.31.0
    Uninstalling asyncpg-0.31.0:
      Successfully uninstalled asyncpg-0.31.0
Successfully installed asyncpg-0.29.0

[notice] A new release of pip is available: 25.0.1 -> 25.3
[notice] To update, run: python.exe -m pip install --upgrade pip
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook (main)
$ cd app
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$  pip install -r requirements.txt
Requirement already satisfied: fastapi==0.115.6 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 2)) (0.115.6)
Requirement already satisfied: uvicorn==0.34.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.34.0)
Requirement already satisfied: sqlalchemy==2.0.36 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 7)) (2.0.36)
Collecting psycopg2-binary==2.9.11 (from -r requirements.txt (line 8))
  Using cached psycopg2_binary-2.9.11-cp312-cp312-win_amd64.whl.metadata (5.1 kB)
Requirement already satisfied: alembic==1.14.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 9)) (1.14.1)
Requirement already satisfied: asyncpg in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 10)) (0.29.0)
Requirement already satisfied: aiosqlite==0.20.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 11)) (0.20.0)
Requirement already satisfied: qdrant-client==1.12.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 14)) (1.12.1)
Requirement already satisfied: langchain-google-genai in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 17)) (2.0.10)
Requirement already satisfied: google-generativeai in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 18)) (0.8.6)
Requirement already satisfied: langchain==0.3.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 19)) (0.3.9)
Requirement already satisfied: langchain-community==0.3.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 20)) (0.3.9)
Requirement already satisfied: langchain-text-splitters==0.3.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 21)) (0.3.2)
Requirement already satisfied: tiktoken in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 22)) (0.12.0)
Requirement already satisfied: passlib==1.7.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from passlib[bcrypt]==1.7.4->-r requirements.txt (line 25)) (1.7.4)
Requirement already satisfied: python-jose==3.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (3.3.0)
Requirement already satisfied: python-dotenv==1.0.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 29)) (1.0.1)
Requirement already satisfied: httpx==0.28.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 32)) (0.28.1)
Requirement already satisfied: beautifulsoup4==4.13.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 33)) (4.13.0)
Requirement already satisfied: markdown==3.10.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 34)) (3.10)
Requirement already satisfied: pyjwt==2.10.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 35)) (2.10.1)
Requirement already satisfied: fastapi-cors==0.0.6 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 36)) (0.0.6)
Requirement already satisfied: python-multipart==0.0.20 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 37)) (0.0.20)
Requirement already satisfied: pytest==8.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 40)) (8.4.0)
Requirement already satisfied: pytest-asyncio==0.24.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 41)) (0.24.0)
Requirement already satisfied: requests==2.32.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 42)) (2.32.3)
Requirement already satisfied: pydantic[email] in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 4)) (2.12.5)
Requirement already satisfied: starlette<0.42.0,>=0.40.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi==0.115.6->-r requirements.txt (line 2)) (0.41.3)
Requirement already satisfied: typing-extensions>=4.8.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi==0.115.6->-r requirements.txt (line 2)) (4.15.0)
Requirement already satisfied: click>=7.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn==0.34.0->uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (8.3.1)
Requirement already satisfied: h11>=0.8 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn==0.34.0->uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.16.0)
Requirement already satisfied: greenlet!=0.4.17 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from sqlalchemy==2.0.36->-r requirements.txt (line 7)) (3.3.0)
Requirement already satisfied: Mako in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from alembic==1.14.1->-r requirements.txt (line 9)) (1.3.10)
Requirement already satisfied: grpcio>=1.41.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 14)) (1.76.0)
Requirement already satisfied: grpcio-tools>=1.41.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 14)) (1.71.2)
Requirement already satisfied: numpy>=1.26 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 14)) (2.4.0)
Requirement already satisfied: portalocker<3.0.0,>=2.7.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 14)) (2.10.1)
Requirement already satisfied: urllib3<3,>=1.26.14 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 14)) (2.6.2)
Requirement already satisfied: PyYAML>=5.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 19)) (6.0.3)
Requirement already satisfied: aiohttp<4.0.0,>=3.8.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 19)) (3.13.2)
Requirement already satisfied: langchain-core<0.4.0,>=0.3.21 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 19)) (0.3.63)
Requirement already satisfied: langsmith<0.2.0,>=0.1.17 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 19)) (0.1.147)
Requirement already satisfied: tenacity!=8.4.0,<10,>=8.1.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 19)) (9.1.2)
Requirement already satisfied: dataclasses-json<0.7,>=0.5.7 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 20)) (0.6.7)
Requirement already satisfied: httpx-sse<0.5.0,>=0.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 20)) (0.4.3)
Requirement already satisfied: pydantic-settings<3.0.0,>=2.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 20)) (2.12.0)
Requirement already satisfied: ecdsa!=0.15 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (0.19.1)
Requirement already satisfied: rsa in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (4.9.1)
Requirement already satisfied: pyasn1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from 
python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (0.6.1)
Requirement already satisfied: anyio in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 32)) (4.12.0)
Requirement already satisfied: certifi in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 32)) (2025.11.12)
Requirement already satisfied: httpcore==1.* in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 32)) (1.0.9)
Requirement already satisfied: idna in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 32)) (3.11)
Requirement already satisfied: soupsieve>1.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from beautifulsoup4==4.13.0->-r requirements.txt (line 33)) (2.8.1)
Requirement already satisfied: environs>=9.5.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi-cors==0.0.6->-r requirements.txt (line 36)) (14.5.0)
Requirement already satisfied: colorama>=0.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 40)) (0.4.6)
Requirement already satisfied: iniconfig>=1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages 
(from pytest==8.4.0->-r requirements.txt (line 40)) (2.3.0)
Requirement already satisfied: packaging>=20 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 40)) (24.2)
Requirement already satisfied: pluggy<2,>=1.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 40)) (1.6.0)
Requirement already satisfied: pygments>=2.7.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 40)) (2.19.2)
Requirement already satisfied: charset-normalizer<4,>=2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from requests==2.32.3->-r requirements.txt (line 42)) (3.4.4)
Requirement already satisfied: bcrypt>=3.1.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from passlib[bcrypt]==1.7.4->-r requirements.txt (line 25)) (5.0.0)
Requirement already satisfied: cryptography>=3.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (46.0.3)
Requirement already satisfied: httptools>=0.6.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.7.1)
Requirement already satisfied: watchfiles>=0.13 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (1.1.1)
Requirement already satisfied: websockets>=10.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (15.0.1)
Requirement already satisfied: annotated-types>=0.6.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (0.7.0)
Requirement already satisfied: pydantic-core==2.41.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (2.41.5)
Requirement already satisfied: typing-inspection>=0.4.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (0.4.2)
Requirement already satisfied: email-validator>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (2.3.0)
Requirement already satisfied: filetype<2.0.0,>=1.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-google-genai->-r requirements.txt (line 17)) (1.2.0)
Requirement already satisfied: google-ai-generativelanguage==0.6.15 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (0.6.15)
Requirement already satisfied: google-api-core in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (2.28.1)
Requirement already satisfied: google-api-python-client in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (2.187.0)
Requirement already satisfied: google-auth>=2.15.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (2.45.0)
Requirement already satisfied: protobuf in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (5.29.5)
Requirement already satisfied: tqdm in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 18)) (4.67.1)
Requirement already satisfied: proto-plus<2.0.0dev,>=1.22.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-ai-generativelanguage==0.6.15->google-generativeai->-r requirements.txt (line 18)) (1.27.0)
Requirement already satisfied: regex>=2022.1.18 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from tiktoken->-r requirements.txt (line 22)) (2025.11.3)
Requirement already satisfied: aiohappyeyeballs>=2.5.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (2.6.1)
Requirement already satisfied: aiosignal>=1.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (1.4.0)
Requirement already satisfied: attrs>=17.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (25.4.0)
Requirement already satisfied: frozenlist>=1.1.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (1.8.0)
Requirement already satisfied: multidict<7.0,>=4.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (6.7.0)
Requirement already satisfied: propcache>=0.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (0.4.1)
Requirement already satisfied: yarl<2.0,>=1.17.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 19)) (1.22.0)
Requirement already satisfied: cffi>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from cryptography>=3.4.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (2.0.0)
Requirement already satisfied: marshmallow<4.0.0,>=3.18.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 20)) (3.26.1)
Requirement already satisfied: typing-inspect<1,>=0.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 20)) (0.9.0)
Requirement already satisfied: six>=1.9.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from ecdsa!=0.15->python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (1.17.0)   
Requirement already satisfied: dnspython>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from email-validator>=2.0.0->pydantic[email]->-r requirements.txt (line 4)) (2.8.0)
Requirement already satisfied: googleapis-common-protos<2.0.0,>=1.56.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-core->google-generativeai->-r requirements.txt (line 18)) (1.72.0)   
Requirement already satisfied: cachetools<7.0,>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-auth>=2.15.0->google-generativeai->-r requirements.txt (line 18)) (6.2.4)
Requirement already satisfied: pyasn1-modules>=0.2.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-auth>=2.15.0->google-generativeai->-r requirements.txt (line 18)) (0.4.2)
Requirement already satisfied: setuptools in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from grpcio-tools>=1.41.0->qdrant-client==1.12.1->-r requirements.txt (line 14)) (80.9.0)
Requirement already satisfied: h2<5,>=3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 14)) (4.3.0)
Requirement already satisfied: jsonpatch<2.0,>=1.33 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-core<0.4.0,>=0.3.21->langchain==0.3.9->-r requirements.txt (line 19)) (1.33)
Requirement already satisfied: orjson<4.0.0,>=3.9.14 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langsmith<0.2.0,>=0.1.17->langchain==0.3.9->-r requirements.txt (line 19)) (3.11.5)
Requirement already satisfied: requests-toolbelt<2.0.0,>=1.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langsmith<0.2.0,>=0.1.17->langchain==0.3.9->-r requirements.txt (line 19)) (1.0.0)      
Requirement already satisfied: pywin32>=226 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages 
(from portalocker<3.0.0,>=2.7.0->qdrant-client==1.12.1->-r requirements.txt (line 14)) (311)
Requirement already satisfied: httplib2<1.0.0,>=0.19.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 18)) (0.31.0)
Requirement already satisfied: google-auth-httplib2<1.0.0,>=0.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 18)) (0.3.0)Requirement already satisfied: uritemplate<5,>=3.0.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 18)) (4.2.0)
Requirement already satisfied: MarkupSafe>=0.9.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from Mako->alembic==1.14.1->-r requirements.txt (line 9)) (3.0.3)
Requirement already satisfied: pycparser in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from cffi>=2.0.0->cryptography>=3.4.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 26)) (2.23)     
Requirement already satisfied: grpcio-status<2.0.0,>=1.33.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-core[grpc]!=2.0.*,!=2.1.*,!=2.10.*,!=2.2.*,!=2.3.*,!=2.4.*,!=2.5.*,!=2.6.*,!=2.7.*,!=2.8.*,!=2.9.*,<3.0.0dev,>=1.34.1->google-ai-generativelanguage==0.6.15->google-generativeai->-r requirements.txt (line 18)) (1.71.2)
Requirement already satisfied: hyperframe<7,>=6.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from h2<5,>=3->httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 14)) (6.1.0)        
Requirement already satisfied: hpack<5,>=4.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from h2<5,>=3->httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 14)) (4.1.0)
Requirement already satisfied: pyparsing<4,>=3.0.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httplib2<1.0.0,>=0.19.0->google-api-python-client->google-generativeai->-r requirements.txt (line 18)) (3.2.5)
Requirement already satisfied: jsonpointer>=1.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from jsonpatch<2.0,>=1.33->langchain-core<0.4.0,>=0.3.21->langchain==0.3.9->-r requirements.txt (line 19)) (3.0.0)
Requirement already satisfied: mypy-extensions>=0.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from typing-inspect<1,>=0.4.0->dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 20)) (1.1.0)
Using cached psycopg2_binary-2.9.11-cp312-cp312-win_amd64.whl (2.7 MB)
Installing collected packages: psycopg2-binary
Successfully installed psycopg2-binary-2.9.11

[notice] A new release of pip is available: 25.0.1 -> 25.3
[notice] To update, run: python.exe -m pip install --upgrade pip
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ uvicorn main:app --reload
INFO:     Will watch for changes in these directories: ['D:\\Physical-AI-Humanoid-Robotics-Textbook\\app']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [5476] using WatchFiles
Process SpawnProcess-1:
Traceback (most recent call last):
  File "C:\Program Files\Python312\Lib\multiprocessing\process.py", line 314, in _bootstrap
    self.run()
  File "C:\Program Files\Python312\Lib\multiprocessing\process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\_subprocess.py", line 80, in subprocess_started
    target(sockets=sockets)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 66, in run    
    return asyncio.run(self.serve(sockets=sockets))
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\runners.py", line 195, in run
    return runner.run(main)
           ^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\runners.py", line 118, in run
    return self._loop.run_until_complete(task)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\base_events.py", line 691, in run_until_complete
    return future.result()
           ^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 70, in serve  
    await self._serve(sockets)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 77, in _serve 
    config.load()
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\config.py", line 435, in load  
    self.loaded_app = import_from_string(self.app)
                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\importer.py", line 19, in import_from_string
    module = importlib.import_module(module_str)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\importlib\__init__.py", line 90, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1331, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 935, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 999, in exec_module
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\app\main.py", line 12, in <module>
    from app.database import init_db
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\app\database.py", line 23, in <module>
    engine = create_async_engine(DATABASE_URL, echo=True, connect_args={"ssl": "require"})
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\ext\asyncio\engine.py", line 121, in create_async_engine
    return AsyncEngine(sync_engine)
           ^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\ext\asyncio\engine.py", line 1031, in __init__
    raise exc.InvalidRequestError(
sqlalchemy.exc.InvalidRequestError: The asyncio extension requires an async driver to be used. The loaded 'psycopg2' is not async.
INFO:     Stopping reloader process [5476]
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ pip uninstall psycopg2-binary psycopg2 -y
Found existing installation: psycopg2-binary 2.9.11
Uninstalling psycopg2-binary-2.9.11:
  Successfully uninstalled psycopg2-binary-2.9.11
WARNING: Skipping psycopg2 as it is not installed.
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ cp requirements.txt requirements.txt.backup
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ sed -i '/psycopg2-binary/d' requirements.txt
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ pip install -r requirements.txt
Requirement already satisfied: fastapi==0.115.6 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 2)) (0.115.6)
Requirement already satisfied: uvicorn==0.34.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.34.0)
Requirement already satisfied: sqlalchemy==2.0.36 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 7)) (2.0.36)
Requirement already satisfied: alembic==1.14.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 8)) (1.14.1)
Requirement already satisfied: asyncpg in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 9)) (0.29.0)
Requirement already satisfied: aiosqlite==0.20.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 10)) (0.20.0)
Requirement already satisfied: qdrant-client==1.12.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 13)) (1.12.1)
Requirement already satisfied: langchain-google-genai in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 16)) (2.0.10)
Requirement already satisfied: google-generativeai in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 17)) (0.8.6)
Requirement already satisfied: langchain==0.3.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 18)) (0.3.9)
Requirement already satisfied: langchain-community==0.3.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 19)) (0.3.9)
Requirement already satisfied: langchain-text-splitters==0.3.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 20)) (0.3.2)
Requirement already satisfied: tiktoken in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 21)) (0.12.0)
Requirement already satisfied: passlib==1.7.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from passlib[bcrypt]==1.7.4->-r requirements.txt (line 24)) (1.7.4)
Requirement already satisfied: python-jose==3.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (3.3.0)
Requirement already satisfied: python-dotenv==1.0.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 28)) (1.0.1)
Requirement already satisfied: httpx==0.28.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 31)) (0.28.1)
Requirement already satisfied: beautifulsoup4==4.13.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 32)) (4.13.0)
Requirement already satisfied: markdown==3.10.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 33)) (3.10)
Requirement already satisfied: pyjwt==2.10.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 34)) (2.10.1)
Requirement already satisfied: fastapi-cors==0.0.6 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 35)) (0.0.6)
Requirement already satisfied: python-multipart==0.0.20 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 36)) (0.0.20)
Requirement already satisfied: pytest==8.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 39)) (8.4.0)
Requirement already satisfied: pytest-asyncio==0.24.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 40)) (0.24.0)
Requirement already satisfied: requests==2.32.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 41)) (2.32.3)
Requirement already satisfied: pydantic[email] in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from -r requirements.txt (line 4)) (2.12.5)
Requirement already satisfied: starlette<0.42.0,>=0.40.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi==0.115.6->-r requirements.txt (line 2)) (0.41.3)
Requirement already satisfied: typing-extensions>=4.8.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi==0.115.6->-r requirements.txt (line 2)) (4.15.0)
Requirement already satisfied: click>=7.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn==0.34.0->uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (8.3.1)
Requirement already satisfied: h11>=0.8 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn==0.34.0->uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.16.0)
Requirement already satisfied: greenlet!=0.4.17 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from sqlalchemy==2.0.36->-r requirements.txt (line 7)) (3.3.0)
Requirement already satisfied: Mako in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from alembic==1.14.1->-r requirements.txt (line 8)) (1.3.10)
Requirement already satisfied: grpcio>=1.41.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 13)) (1.76.0)
Requirement already satisfied: grpcio-tools>=1.41.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 13)) (1.71.2)
Requirement already satisfied: numpy>=1.26 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 13)) (2.4.0)
Requirement already satisfied: portalocker<3.0.0,>=2.7.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 13)) (2.10.1)
Requirement already satisfied: urllib3<3,>=1.26.14 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from qdrant-client==1.12.1->-r requirements.txt (line 13)) (2.6.2)
Requirement already satisfied: PyYAML>=5.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 18)) (6.0.3)
Requirement already satisfied: aiohttp<4.0.0,>=3.8.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 18)) (3.13.2)
Requirement already satisfied: langchain-core<0.4.0,>=0.3.21 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 18)) (0.3.63)
Requirement already satisfied: langsmith<0.2.0,>=0.1.17 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 18)) (0.1.147)
Requirement already satisfied: tenacity!=8.4.0,<10,>=8.1.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain==0.3.9->-r requirements.txt (line 18)) (9.1.2)
Requirement already satisfied: dataclasses-json<0.7,>=0.5.7 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 19)) (0.6.7)
Requirement already satisfied: httpx-sse<0.5.0,>=0.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 19)) (0.4.3)
Requirement already satisfied: pydantic-settings<3.0.0,>=2.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-community==0.3.9->-r requirements.txt (line 19)) (2.12.0)
Requirement already satisfied: ecdsa!=0.15 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (0.19.1)
Requirement already satisfied: rsa in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (4.9.1)
Requirement already satisfied: pyasn1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from 
python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (0.6.1)
Requirement already satisfied: anyio in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 31)) (4.12.0)
Requirement already satisfied: certifi in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 31)) (2025.11.12)
Requirement already satisfied: httpcore==1.* in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 31)) (1.0.9)
Requirement already satisfied: idna in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx==0.28.1->-r requirements.txt (line 31)) (3.11)
Requirement already satisfied: soupsieve>1.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from beautifulsoup4==4.13.0->-r requirements.txt (line 32)) (2.8.1)
Requirement already satisfied: environs>=9.5.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from fastapi-cors==0.0.6->-r requirements.txt (line 35)) (14.5.0)
Requirement already satisfied: colorama>=0.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 39)) (0.4.6)
Requirement already satisfied: iniconfig>=1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages 
(from pytest==8.4.0->-r requirements.txt (line 39)) (2.3.0)
Requirement already satisfied: packaging>=20 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 39)) (24.2)
Requirement already satisfied: pluggy<2,>=1.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 39)) (1.6.0)
Requirement already satisfied: pygments>=2.7.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pytest==8.4.0->-r requirements.txt (line 39)) (2.19.2)
Requirement already satisfied: charset-normalizer<4,>=2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from requests==2.32.3->-r requirements.txt (line 41)) (3.4.4)
Requirement already satisfied: bcrypt>=3.1.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from passlib[bcrypt]==1.7.4->-r requirements.txt (line 24)) (5.0.0)
Requirement already satisfied: cryptography>=3.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (46.0.3)
Requirement already satisfied: httptools>=0.6.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (0.7.1)
Requirement already satisfied: watchfiles>=0.13 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (1.1.1)
Requirement already satisfied: websockets>=10.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from uvicorn[standard]==0.34.0->-r requirements.txt (line 3)) (15.0.1)
Requirement already satisfied: annotated-types>=0.6.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (0.7.0)
Requirement already satisfied: pydantic-core==2.41.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (2.41.5)
Requirement already satisfied: typing-inspection>=0.4.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (0.4.2)
Requirement already satisfied: email-validator>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from pydantic[email]->-r requirements.txt (line 4)) (2.3.0)
Requirement already satisfied: filetype<2.0.0,>=1.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-google-genai->-r requirements.txt (line 16)) (1.2.0)
Requirement already satisfied: google-ai-generativelanguage==0.6.15 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (0.6.15)
Requirement already satisfied: google-api-core in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (2.28.1)
Requirement already satisfied: google-api-python-client in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (2.187.0)
Requirement already satisfied: google-auth>=2.15.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (2.45.0)
Requirement already satisfied: protobuf in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (5.29.5)
Requirement already satisfied: tqdm in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-generativeai->-r requirements.txt (line 17)) (4.67.1)
Requirement already satisfied: proto-plus<2.0.0dev,>=1.22.3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-ai-generativelanguage==0.6.15->google-generativeai->-r requirements.txt (line 17)) (1.27.0)
Requirement already satisfied: regex>=2022.1.18 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from tiktoken->-r requirements.txt (line 21)) (2025.11.3)
Requirement already satisfied: aiohappyeyeballs>=2.5.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (2.6.1)
Requirement already satisfied: aiosignal>=1.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (1.4.0)
Requirement already satisfied: attrs>=17.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (25.4.0)
Requirement already satisfied: frozenlist>=1.1.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (1.8.0)
Requirement already satisfied: multidict<7.0,>=4.5 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (6.7.0)
Requirement already satisfied: propcache>=0.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (0.4.1)
Requirement already satisfied: yarl<2.0,>=1.17.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from aiohttp<4.0.0,>=3.8.3->langchain==0.3.9->-r requirements.txt (line 18)) (1.22.0)
Requirement already satisfied: cffi>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from cryptography>=3.4.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (2.0.0)
Requirement already satisfied: marshmallow<4.0.0,>=3.18.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 19)) (3.26.1)
Requirement already satisfied: typing-inspect<1,>=0.4.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 19)) (0.9.0)
Requirement already satisfied: six>=1.9.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from ecdsa!=0.15->python-jose==3.3.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (1.17.0)   
Requirement already satisfied: dnspython>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from email-validator>=2.0.0->pydantic[email]->-r requirements.txt (line 4)) (2.8.0)
Requirement already satisfied: googleapis-common-protos<2.0.0,>=1.56.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-core->google-generativeai->-r requirements.txt (line 17)) (1.72.0)   
Requirement already satisfied: cachetools<7.0,>=2.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-auth>=2.15.0->google-generativeai->-r requirements.txt (line 17)) (6.2.4)
Requirement already satisfied: pyasn1-modules>=0.2.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-auth>=2.15.0->google-generativeai->-r requirements.txt (line 17)) (0.4.2)
Requirement already satisfied: setuptools in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from grpcio-tools>=1.41.0->qdrant-client==1.12.1->-r requirements.txt (line 13)) (80.9.0)
Requirement already satisfied: h2<5,>=3 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 13)) (4.3.0)
Requirement already satisfied: jsonpatch<2.0,>=1.33 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langchain-core<0.4.0,>=0.3.21->langchain==0.3.9->-r requirements.txt (line 18)) (1.33)
Requirement already satisfied: orjson<4.0.0,>=3.9.14 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langsmith<0.2.0,>=0.1.17->langchain==0.3.9->-r requirements.txt (line 18)) (3.11.5)
Requirement already satisfied: requests-toolbelt<2.0.0,>=1.0.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from langsmith<0.2.0,>=0.1.17->langchain==0.3.9->-r requirements.txt (line 18)) (1.0.0)      
Requirement already satisfied: pywin32>=226 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages 
(from portalocker<3.0.0,>=2.7.0->qdrant-client==1.12.1->-r requirements.txt (line 13)) (311)
Requirement already satisfied: httplib2<1.0.0,>=0.19.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 17)) (0.31.0)
Requirement already satisfied: google-auth-httplib2<1.0.0,>=0.2.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 17)) (0.3.0)Requirement already satisfied: uritemplate<5,>=3.0.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-python-client->google-generativeai->-r requirements.txt (line 17)) (4.2.0)
Requirement already satisfied: MarkupSafe>=0.9.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from Mako->alembic==1.14.1->-r requirements.txt (line 8)) (3.0.3)
Requirement already satisfied: pycparser in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from cffi>=2.0.0->cryptography>=3.4.0->python-jose[cryptography]==3.3.0->-r requirements.txt (line 25)) (2.23)     
Requirement already satisfied: grpcio-status<2.0.0,>=1.33.2 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from google-api-core[grpc]!=2.0.*,!=2.1.*,!=2.10.*,!=2.2.*,!=2.3.*,!=2.4.*,!=2.5.*,!=2.6.*,!=2.7.*,!=2.8.*,!=2.9.*,<3.0.0dev,>=1.34.1->google-ai-generativelanguage==0.6.15->google-generativeai->-r requirements.txt (line 17)) (1.71.2)
Requirement already satisfied: hyperframe<7,>=6.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from h2<5,>=3->httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 13)) (6.1.0)        
Requirement already satisfied: hpack<5,>=4.1 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from h2<5,>=3->httpx[http2]>=0.20.0->qdrant-client==1.12.1->-r requirements.txt (line 13)) (4.1.0)
Requirement already satisfied: pyparsing<4,>=3.0.4 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from httplib2<1.0.0,>=0.19.0->google-api-python-client->google-generativeai->-r requirements.txt (line 17)) (3.2.5)
Requirement already satisfied: jsonpointer>=1.9 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from jsonpatch<2.0,>=1.33->langchain-core<0.4.0,>=0.3.21->langchain==0.3.9->-r requirements.txt (line 18)) (3.0.0)
Requirement already satisfied: mypy-extensions>=0.3.0 in d:\physical-ai-humanoid-robotics-textbook\.venv\lib\site-packages (from typing-inspect<1,>=0.4.0->dataclasses-json<0.7,>=0.5.7->langchain-community==0.3.9->-r requirements.txt (line 19)) (1.1.0)

[notice] A new release of pip is available: 25.0.1 -> 25.3
[notice] To update, run: python.exe -m pip install --upgrade pip
((.venv) ) 
pc@DESKTOP-GC7T4DM MINGW64 /d/Physical-AI-Humanoid-Robotics-Textbook/app (main)
$ uvicorn main:app --reload
INFO:     Will watch for changes in these directories: ['D:\\Physical-AI-Humanoid-Robotics-Textbook\\app']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12100] using WatchFiles
Process SpawnProcess-1:
Traceback (most recent call last):
  File "C:\Program Files\Python312\Lib\multiprocessing\process.py", line 314, in _bootstrap
    self.run()
  File "C:\Program Files\Python312\Lib\multiprocessing\process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\_subprocess.py", line 80, in subprocess_started
    target(sockets=sockets)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 66, in run    
    return asyncio.run(self.serve(sockets=sockets))
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\runners.py", line 195, in run
    return runner.run(main)
           ^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\runners.py", line 118, in run
    return self._loop.run_until_complete(task)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\asyncio\base_events.py", line 691, in run_until_complete
    return future.result()
           ^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 70, in serve  
    await self._serve(sockets)
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\server.py", line 77, in _serve 
    config.load()
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\config.py", line 435, in load  
    self.loaded_app = import_from_string(self.app)
                      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\importer.py", line 22, in import_from_string
    raise exc from None
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\uvicorn\importer.py", line 19, in import_from_string
    module = importlib.import_module(module_str)
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "C:\Program Files\Python312\Lib\importlib\__init__.py", line 90, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<frozen importlib._bootstrap>", line 1387, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1360, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1331, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 935, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 999, in exec_module
  File "<frozen importlib._bootstrap>", line 488, in _call_with_frames_removed
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\app\main.py", line 12, in <module>
    from app.database import init_db
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\app\database.py", line 23, in <module>
    engine = create_async_engine(DATABASE_URL, echo=True, connect_args={"ssl": "require"})
             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\ext\asyncio\engine.py", line 120, in create_async_engine
    sync_engine = _create_engine(url, **kw)
                  ^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<string>", line 2, in create_engine
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\util\deprecations.py", line 
281, in warned
    return fn(*args, **kwargs)  # type: ignore[no-any-return]
           ^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\engine\create.py", line 599, in create_engine
    dbapi = dbapi_meth(**dbapi_args)
            ^^^^^^^^^^^^^^^^^^^^^^^^
  File "D:\Physical-AI-Humanoid-Robotics-Textbook\.venv\Lib\site-packages\sqlalchemy\dialects\postgresql\psycopg2.py", line 690, in import_dbapi
    import psycopg2
ModuleNotFoundError: No module named 'psycopg2'
