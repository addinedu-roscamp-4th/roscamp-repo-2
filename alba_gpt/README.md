### 📁 파일 구조

~~~
AlbaGPT
├── AlbaGPT_communication 
│   ├── AlbaGPT_UDP.py : AlbaGPT UDP 서버 
├── AlbaGPT_LLM
│   ├── Alba_LLM_templates.py : AlbaGPT LLM 탬플릿 모음
│   ├── contents
│   │   ├── example : AlbaBot에게 주어지는 각 prompt에 대한 답변 예시
│   │   │   ├── greetings_example.csv
│   │   │   ├── task_example.csv
│   │   │   └── work_example.csv
│   │   ├── image : AlbaBot에게부터 받은 사진
│   │   │   └── 1ef8387c84da45439b5f3d12b807dcf4_2025-04-30 12-01-50.jpg
│   │   └── response : AlbaGPT 모델의 출력 결과
│   │       ├── birthday_response.json
│   │       ├── camera_on_response.json
│   │       ├── cleaning_response.json
│   │       ├── emergency_response.json
│   │       ├── greetings_response.json
│   │       └── serving_response.json
│   ├── main.py : AlbaGPT 메인 코드
│   └── __pycache__
│       └── Alba_LLM_templates.cpython-311.pyc
├── AlbaGPT_Vision
├── README.md
└── requirements.txt : 가상환경 설정
~~~
