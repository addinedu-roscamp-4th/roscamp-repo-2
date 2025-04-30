from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.chat_models import ChatOpenAI
from langchain_teddynote.models import MultiModal
from dotenv import load_dotenv

import json
import csv
import cv2
import os
import sys
import numpy as np
import uuid
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

load_dotenv()

alba_work_type_list = ["cleaning", "serving", "birthday", "emergency"]
alba_task_type_list = alba_work_type_list + ["greetings", "camera on"]

def alba_task_discriminator(user_query, alba_task_type_list=alba_task_type_list):
    """
    알바봇에게 입력된 프롬프트가 어떤 type의 명령인지 구분해주는 함수입니다.
    
    Returns : 
        cleaning, serving, birthday, emergency, greetings, camera on, none 중 하나
    """
    task_example_csv_path = './contents/example/task_example.csv' # 유저 프롬프트에 따른 분류된 task를 정리한 csv 파일
    fields = []
    task_example_list = []

    with open(task_example_csv_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        fields = next(csvreader)
    
        for row in csvreader:
            task_example_list.append({
                "prompt": row[0],
                "result": row[1]
            })
    
    task_example = "\n".join(
        f'{{"prompt": "{task["prompt"]}", "result": "{task["result"]}"}}'
        for task in task_example_list
    )

    discriminator_prompt = """
    당신은 레스토랑에서 업무를 하는 카메라가 장착된 '핑키'라는 이름을 가진 모바일 로봇입니다.
    당신은 오너의 다음 요청을 보고, 그것이 인사(greeting)인지, 작업 명령(task command)인지 구분해야 합니다.

    다음 요청: {user_query}
    
    판단 기준은 다음과 같습니다:

    1. 요청이 인사인 경우 → 정확히 **"greetings"** 라고만 출력하세요.
    2. 요청이 작업 명령일 경우 → 해당 작업이 {alba_task_type_list} 배열 원소 중 무엇인지 판단하고, 해당 태스크 이름만 출력하세요.
    3. 만약 "무엇이 보이냐", "보이는 것을 설명해달라"는 요청이 포함된다면, **camera on**으로 간주하세요. 이 경우 '카메라'라는 단어가 없어도 무조건 camera on으로 판단하세요.
    4. {alba_task_type_list}의 배열에 없는 원소에 대해서는 **"none"**을 출력하세요.

    이때 아래 조건을 반드시 따르세요:
    - 출력은 반드시 **소문자** 단어 하나로만 답변하세요.
    - 추가 설명, 문장, 다른 단어는 절대 포함하지 마세요.
    - 가능한 한 명확하고 단정적으로 판단하세요.
    - 설명, 문장, 구두점은 답변으로 내놓지 않아요.

    정답 예시:
    {task_example}
    """

    discriminator_template = PromptTemplate(
        template=discriminator_prompt,
        input_variables=["user_query", "alba_task_type_list", "task_example"],
    )

    # 객체 생성
    llm = ChatOpenAI(
        temperature=0.3,  # 창의성 (0.0 ~ 2.0)
        max_tokens=2048,  # 최대 토큰수
        model_name="gpt-4o-mini",  # 모델명
    )

    chain = LLMChain(
        llm=llm,
        prompt=discriminator_template
    )

    discriminated_task = chain.invoke({
        "user_query": user_query,
        "alba_task_type_list": alba_task_type_list,
        "task_example": task_example
    })['text']

    discriminated_task = discriminated_task.replace(".", "").strip()
    
    return discriminated_task
"""

"""
def validate_alba_task_discriminator(user_query, alba_task_type_list=alba_task_type_list):
    """
    alba_task_discriminator 함수로 구분한 task가 alba_task_type_list 배열의 원소 중 하나가 맞는 지 한 번 더 판별해주는 함수입니다.

    Returns : 
        alba_task_type_list에 있는 경우 : {discriminated_task}
        alba_task_type_list에 없는 경우 : None
    """
    discriminated_task = alba_task_discriminator(user_query)
    
    if discriminated_task not in alba_task_type_list :
        return None
    else :
        return discriminated_task

def generate_alba_greetings_response(user_query, chat_history, memory):
    """
    alba_task_discriminator 함수로 구분된 task가 'greetings'인 경우 user_query에 대해 대답을 생성하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "pinky_id" : {pinky_id},
            "pinky_task" : greetings,
            "pinky_response" : {alba_greetings_response}
        }}
    """
    greetings_example_csv_path = './contents/example/greetings_example.csv' # greetings 상황에 대한 응답을 정리한 csv 파일
    fields = []
    greetings_example_list = []

    with open(greetings_example_csv_path, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        
        # extracting field names through first row
        fields = next(csvreader)
    
        # extracting each data row one by one
        for row in csvreader:
            greetings_example_list.append({
            "prompt": row[0],
            "pinky_id": row[1],
            "pinky_task": row[2],
            "pinky_response": row[3]
        })
            
    greetings_example = "\n".join(
        f'{{"prompt": "{greetings["prompt"]}", "pinky_id": "{greetings["pinky_id"]}", "pinky_task": "{greetings["pinky_task"]}", "pinky_response": "{greetings["pinky_response"]}"}}'
        for greetings in greetings_example_list
    )

    alba_greetings_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신은 레스토랑 오너의 {user_query}에 대해 친절하고 상냥하게, 1~3줄 이내로 너무 길지 않게 답변하여야 합니다.

    출력 예시:
    {greetings_example}

    아래의 [조건]에 맞게 [탬플릿] 형식으로 JSON 답변을 생성해주세요:

    [조건]
    1. pinky_id는 "X번 핑키!"에서의 X 입니다. 이때 X가 없으면 pinky_id는 비워두세요.
    2. pinky_task는 무조건 string형 "greetings" 입니다.
    3. pinky_response는 {user_query}에 대한 string형 답변입니다.
    4. 출력은 반드시 JSON 형식의 텍스트만 반환해야 합니다.
    5. 절대 마크다운 형식(예: ```json, ``` 또는 ''' 등)을 사용하지 마세요.
    6. JSON 객체만 출력하세요. 문자열 앞뒤 공백 외에는 아무것도 포함하지 마세요.
    7. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.

    [탬플릿] : 반드시 큰따옴표로 감싸서 문자열 형태로 출력해야 합니다. 반드시 문자열 이외의 형식은 출력하지 않습니다. '''json 같은 마크다운은 절대 포함하면 안됩니다.
    {{
        "pinky_id": "",
        "pinky_task": "",
        "pinky_response": "",
    }}
    """

    alba_greetings_template = PromptTemplate(
        template=alba_greetings_prompt,
        input_variables=["user_query", "chat_history", "greetings_example"]
    )

    # 객체 생성
    llm = ChatOpenAI(
        temperature=0.3,  # 창의성 (0.0 ~ 2.0)
        max_tokens=2048,  # 최대 토큰수
        model_name="gpt-4o-mini",  # 모델명
    )

    chain = LLMChain(
        llm=llm,
        prompt=alba_greetings_template,
        memory=memory,
        verbose=True
    )

    alba_greetings_response = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "greetings_example": greetings_example
    })['text']

    try:
        alba_greetings_response = json.loads(alba_greetings_response)
    except json.JSONDecodeError:
        raise ValueError(f"❌ Invalid JSON format: {alba_greetings_response}")

    csvfile.close()
    return alba_greetings_response

def generate_alba_work_response(user_query, chat_history, memory, work):
    """
    alba_task_discriminator 함수로 구분된 task가 {alba_work_type_list}의 원소 중 하나인 경우 user_query에 대해 대답을 생성하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "prompt": {user_query},
            "pinky_id": {인식된 핑키 id},
            "pinky_response": {},
            "table_id": {인식된 핑키 id}
        }}
    """
    work_example_csv_path = './contents/example/work_example.csv' # {alba_work_type} 상황에 대한 응답을 정리한 csv 파일
    fields = []
    work_example_list = []

    with open(work_example_csv_path, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        
        # extracting field names through first row
        fields = next(csvreader)
    
        # extracting each data row one by one
        for row in csvreader:
            work_example_list.append({
            "prompt": row[0],
            "pinky_id": row[1],
            "pinky_response": row[2],
            "table_id": row[3]
        })
            
    work_example = "\n".join(
        f'{{"prompt": "{work["prompt"]}", "pinky_id": "{work["pinky_id"]}", "pinky_response": "{work["pinky_response"]}", "table_id": "{work["table_id"]}"}}'
        for work in work_example_list
    )

    alba_work_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신은 레스토랑 오너의 {user_query}에 대해 {chat_history}와 연관이 있다면 참고하여 {work}라는 업무를 수행하는 것입니다.
    당신은 {user_query}에 대해서 1~3줄 이내로 짧고 간결하게 답변하여야 합니다.

    출력 예시:
    {work_example}

    아래의 [조건]에 맞게 [탬플릿] 형식으로 JSON 답변을 생성해주세요:

    [조건]
    1. pinky_id는 "X번 핑키!"에서의 X 입니다. 이때 X가 없으면 pinky_id는 비워두세요.
    2. pinky_task는 무조건 string형 {work} 입니다.
    3. pinky_response는 {user_query}에 대한 string형 답변입니다.
    4. table_id는 "X번 테이블"에서의 X 입니다. 반드시 X 이외의 다른 문장이 오면 안됩니다. 이때 X가 없으면 table_id는 비워두세요.
    5. 출력은 반드시 JSON 형식의 텍스트만 반환해야 합니다.
    6. 절대 마크다운 형식(예: ```json, ``` 또는 ''' 등)을 사용하지 마세요.
    7. JSON 객체만 출력하세요. 문자열 앞뒤 공백 외에는 아무것도 포함하지 마세요.
    8. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.

    [탬플릿] : 반드시 큰따옴표로 감싸서 문자열 형태로 출력해야 합니다. 반드시 문자열 이외의 형식은 출력하지 않습니다. '''json 같은 마크다운은 절대 포함하면 안됩니다.
    {{
        "pinky_id": "",
        "pinky_task": "",
        "pinky_response": "",
        "table_id": ""
    }}
    """

    alba_work_template = PromptTemplate(
        template=alba_work_prompt,
        input_variables=["user_query", "chat_history", "work_example", "work"]
    )

    # 객체 생성
    llm = ChatOpenAI(
        temperature=0.3,  # 창의성 (0.0 ~ 2.0)
        max_tokens=2048,  # 최대 토큰수
        model_name="gpt-4o-mini",  # 모델명
    )

    chain = LLMChain(
        llm=llm,
        prompt=alba_work_template,
        memory=memory,
        verbose=True
    )

    alba_work_response = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "work" : work,
        "work_example" : work_example
    })['text']


    try:
        alba_work_response = json.loads(alba_work_response)
    except json.JSONDecodeError:
        raise ValueError(f"❌ Invalid JSON format: {alba_work_response}")

    return alba_work_response 

def generate_alba_camera_on_response(user_query, chat_history, recieved_image):
    """
    alba_task_discriminator 함수로 구분된 task가 'camera on'인 경우 해당 알바 봇으로부터 수신되는 영상의 이미지 프레임을 토대로 상황을 해석하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "pinky_id" : {pinky_id},
            "pinky_task" : greetings,
            "pinky_response" : {alba_greetings_response}
        }}
    """
    recieved_image_dir = './contents/image'

    if not os.path.exists(recieved_image_dir) :
        os.makedirs(recieved_image_dir)

    recieved_image_path = os.path.join(recieved_image_dir, str(uuid.uuid4().hex) + '_' + time.strftime('%Y-%m-%d %H-%M-%S') + '.jpg')

    if recieved_image:
        np_data = np.frombuffer(recieved_image, dtype=np.uint8)
        decoded_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        cv2.imwrite(recieved_image_path, decoded_image)
        print(f"⭕️ Image successfully saved to {recieved_image_path}")

        user_prompt = f"""
        {user_query}
        """

        multimodal_system_prompt = """
        당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
        지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

        당신의 임무는 레스토랑 오너의 질문에 대해 친절하고 상냥하게 답변하는 것입니다.
        당신은 박스 안에 그려진 객체에 대해서 설명해야하고, 또한 주변 환경에 대해서도 설명 하여야 합니다.
        
        아래의 [조건]에 맞게 [탬플릿] 형식으로 JSON 답변을 생성해주세요:
        '''json 같은 마크다운은 절대 포함하면 안됩니다.

        [조건]
        1. pinky_id는 "X번 핑키!"에서의 X 입니다. 이때 X가 없으면 pinky_id는 비워두세요.
        2. pinky_task는 무조건 string형 "camera on" 입니다.
        3. pinky_response는 {user_prompt}에 대한 string형 답변입니다.
        4. 출력은 반드시 JSON 형식의 텍스트만 반환해야 합니다.
        5. 절대 마크다운 형식(예: ```json, ``` 또는 ''' 등)을 사용하지 마세요.
        6. JSON 객체만 출력하세요. 문자열 앞뒤 공백 외에는 아무것도 포함하지 마세요.
        7. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.

        [탬플릿] : 반드시 큰따옴표로 감싸서 문자열 형태로 출력해야 합니다. 반드시 문자열 이외의 형식은 출력하지 않습니다. '''json 같은 마크다운은 절대 포함하면 안됩니다.
        {{
            "pinky_id": "",
            "pinky_task": "",
            "pinky_response": "",
        }}
        """

        # 객체 생성
        llm = ChatOpenAI(
            temperature=0.7,  # 창의성 (0.0 ~ 2.0)
            max_tokens=2048,  # 최대 토큰수
            model_name="gpt-4o",  # 모델명
        )

        multimodal_llm = MultiModal(llm, system_prompt=multimodal_system_prompt, user_prompt=user_prompt)

        alba_camera_on_response = multimodal_llm.invoke(recieved_image_path)
        alba_camera_on_response = alba_camera_on_response.strip().strip('"""').strip()
        alba_camera_on_response = alba_camera_on_response.strip().strip('').strip()

        try:
            alba_camera_on_response = json.loads(alba_camera_on_response)
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON format: {alba_camera_on_response}")
        
    else :
        print("❌ Error : 현재 카메라가 켜져있지 않습니다.")
        return 

    return alba_camera_on_response

def save_alba_response(task, alba_response) :
    """
    AlbaGPT가 내놓은 json형 응답을 response 디렉토리에 텍스트 파일로 저장하는 함수입니다. 
    """

    alba_response_dir = './contents/response'

    if not os.path.exists(alba_response_dir) :
        os.makedirs(alba_response_dir)

    if task == "greetings" :
        greetings_file_path = os.path.join(alba_response_dir, 'greetings' + '_response.json')

        with open(greetings_file_path, 'a', encoding="UTF-8") as json_writer:
            json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
            json_writer.write('\n\n')
            print(f"⭕️ Alba response successfully saved to {greetings_file_path}")
        
        json_writer.close()

    elif task == "camera on" :
        camera_on_file_path = os.path.join(alba_response_dir, 'camera_on' + '_response.json')

        with open(camera_on_file_path, 'a', encoding="UTF-8") as json_writer:
            json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
            json_writer.write('\n\n')
            print(f"⭕️ Alba response successfully saved to {camera_on_file_path}")
        
        json_writer.close()
    
    else :
        for work in alba_work_type_list :
            if task == work :
                work_file_path = os.path.join(alba_response_dir, task + '_response.json')

                with open(work_file_path, 'a', encoding="UTF-8") as json_writer:
                    json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
                    json_writer.write('\n\n')
                    print(f"⭕️ Alba response successfully saved to {work_file_path}")
                
                json_writer.close()        
    
    return