from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.chat_models import ChatOpenAI
from langchain_teddynote.models import MultiModal
from dotenv import load_dotenv
from AlbaGPT_main import alba_task_type_list, dynamic_object_list

import json
import csv
import cv2
import os
import numpy as np
import uuid
import time

load_dotenv()

def alba_task_discriminator(user_query, alba_task_type_list=alba_task_type_list):
    """
    알바봇에게 입력된 프롬프트가 어떤 type의 명령인지 구분해주는 함수입니다.
    
    Returns : 
        MAINTENANCE, GREETINGS, CAMERA_ON, none 중 하나
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

    1. 요청이 인사인 경우 → 정확히 **"GREETINGS"** 라고만 출력하세요.
    2. 요청이 작업 명령일 경우 → 해당 작업이 {alba_task_type_list} 배열 원소 중 무엇인지 판단하고, 해당 태스크 이름만 출력하세요.
    3. 만약 "무엇이 보이냐", "보이는 것을 설명해달라"는 요청이 포함된다면, **CAMERA_ON**으로 간주하세요. 이 경우 '카메라'라는 단어가 없어도 무조건 CAMREA_ON으로 판단하세요.
    4. {alba_task_type_list}의 배열에 없는 원소에 대해서는 **"none"**을 출력하세요.

    이때 아래 조건을 반드시 따르세요:
    - 출력은 반드시 **대문자** 단어 하나로만 답변하세요.
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
    alba_task_discriminator 함수로 구분된 task가 'GREETINGS'인 경우 user_query에 대해 대답을 생성하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "pinky_id" : {pinky_id},
            "pinky_task" : GREETINGS,
            "pinky_response" : {alba_greetings_response}
        }}
    """
    greetings_example_csv_path = './contents/example/greetings_example.csv' # GREETINGS 상황에 대한 응답을 정리한 csv 파일
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
        f'{{"prompt": "{GREETINGS["prompt"]}", "pinky_id": "{GREETINGS["pinky_id"]}", "pinky_task": "{GREETINGS["pinky_task"]}", "pinky_response": "{GREETINGS["pinky_response"]}"}}'
        for GREETINGS in greetings_example_list
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
    2. pinky_task는 무조건 string형 "GREETINGS" 입니다.
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

def generate_alba_maintenance_response(user_query, chat_history, memory):
    """
    alba_task_discriminator 함수로 구분된 task가 'MAINTENANCE'인 경우 user_query에 대해 대답을 생성하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "pinky_id": {pinky_id},
            "pinky_task": MAINTENANCE
            "pinky_response": {alba_maintenance_response},
        }}
    """
    maintenance_example_csv_path = './contents/example/maintenance_example.csv' # maintenance 상황에 대한 응답을 정리한 csv 파일
    fields = []
    maintenance_example_list = []

    with open(maintenance_example_csv_path, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        
        # extracting field names through first row
        fields = next(csvreader)
    
        # extracting each data row one by one
        for row in csvreader:
            maintenance_example_list.append({
            "prompt": row[0],
            "pinky_id": row[1],
            "pinky_task": row[2],
            "pinky_response": row[3],
        })
            
    maintenance_example = "\n".join(
        f'{{"prompt": "{maintenance["prompt"]}", "pinky_id": "{maintenance["pinky_id"]}", "pinky_task": "{maintenance["pinky_task"]}", "pinky_response": "{maintenance["pinky_response"]}"}}'
        for maintenance in maintenance_example_list
    )

    alba_maintenance_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신은 레스토랑 오너의 {user_query}에 대해 {chat_history}와 연관이 있다면 참고하여 "MAINTENANCE"라는 업무를 수행하는 것입니다.
    당신은 {user_query}에 대해서 1~3줄 이내로 짧고 간결하게 답변하여야 합니다.

    출력 예시:
    {maintenance_example}

    아래의 [조건]에 맞게 [탬플릿] 형식으로 JSON 답변을 생성해주세요:

    [조건]
    1. pinky_id는 "X번 핑키!"에서의 X 입니다. 이때 X가 없으면 pinky_id는 비워두세요.
    2. pinky_task는 무조건 string형 "MAINTENANCE" 입니다.
    3. pinky_response는 {user_query}에 대한 string형 답변입니다.
    4. 출력은 반드시 JSON 형식의 텍스트만 반환해야 합니다.
    5. 절대 마크다운 형식(예: ```json, ``` 또는 ''' 등)을 사용하지 마세요.
    6. JSON 객체만 출력하세요. 문자열 앞뒤 공백 외에는 아무것도 포함하지 마세요.
    7. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.

    [탬플릿] : 반드시 큰따옴표로 감싸서 문자열 형태로 출력해야 합니다. 반드시 문자열 이외의 형식은 출력하지 않습니다. '''json 같은 마크다운은 절대 포함하면 안됩니다.
    {{
        "pinky_id": "",
        "pinky_task": "MAINTENANCE",
        "pinky_response": "",
    }}
    """

    alba_maintenance_template = PromptTemplate(
        template=alba_maintenance_prompt,
        input_variables=["user_query", "chat_history", "maintenance_example"]
    )

    # 객체 생성
    llm = ChatOpenAI(
        temperature=0.3,  # 창의성 (0.0 ~ 2.0)
        max_tokens=2048,  # 최대 토큰수
        model_name="gpt-4o-mini",  # 모델명
    )

    chain = LLMChain(
        llm=llm,
        prompt=alba_maintenance_template,
        memory=memory,
        verbose=True
    )

    alba_maintenance_response = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "maintenance_example" : maintenance_example
    })['text']


    try:
        alba_maintenance_response = json.loads(alba_maintenance_response)
    except json.JSONDecodeError:
        raise ValueError(f"❌ Invalid JSON format: {alba_maintenance_response}")

    return alba_maintenance_response 

def generate_alba_camera_on_response(chat_history, shared_dict):
    """
    alba_task_discriminator 함수로 구분된 task가 'CAMERA_ON'인 경우 해당 알바 봇으로부터 수신되는 영상의 이미지 프레임을 토대로 상황을 해석하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "pinky_id" : {pinky_id},
            "pinky_task" : CAMERA_ON,
            "pinky_response" : {alba_camera_on_response}
        }}
    """
    recieved_image_dir = './contents/image'

    if not os.path.exists(recieved_image_dir) :
        os.makedirs(recieved_image_dir)

    recieved_image_path = os.path.join(recieved_image_dir, str(uuid.uuid4().hex) + '_' + time.strftime('%Y-%m-%d %H-%M-%S') + '.jpg')
    detected_objects = shared_dict["detected_object"]
    latest_frame = shared_dict["latest_frame"]

    detected_object_list = []

    for detected_object in detected_objects :
        detected = detected_object.categories[0].category_name
        detected_object_list.append(detected)

    print(discriminate_obstacle(detected_object_list))

    np_data = np.frombuffer(latest_frame, dtype=np.uint8)
    decoded_detected_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

    if decoded_detected_image is None:
        raise ValueError("❌ Failed to decode image from decoded_detected_image")

    cv2.imwrite(recieved_image_path, decoded_detected_image)
    print(f"⭕️ Image successfully saved to {recieved_image_path}")

    object_info = ', '.join(detected_object_list)

    obstacle_example_csv_path = './contents/example/obstacle_example.csv' # CAMREA ON 상황에 대한 응답을 정리한 csv 파일
    fields = []
    obstacle_response_example_list = []

    with open(obstacle_example_csv_path, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        
        # extracting field names through first row
        fields = next(csvreader)
    
        # extracting each data row one by one
        for row in csvreader:
            obstacle_response_example_list.append({
            "response": row[0],
        })
            
    obstacle_meetup_example = "\n".join(
        f'{{"response": "{obstacle_response_example["response"]}"}}'
        for obstacle_response_example in obstacle_response_example_list
    )

    user_prompt = f"""
    아래는 감지된 사물 목록입니다: {object_info}
    이 사물들은 경로를 가로막고 있습니다.

    당신의 임무는 다음과 같습니다:
    1. 이 사물들에 대해 반드시 "{object_info} {obstacle_meetup_example}" 와 같은 식으로 재미있게 설명하여야 합니다.
    2. 중복되는 {object_info}에 대해서는 한 번만 대답합니다.
    3. 만약 {object_info}가 null인 경우 "아무 문제 없이 임무를 잘 수행하고 있어요 🤗"와 같은 식으로 설명하여야 합니다.
    4. 답변은 1-3문장 이내로 간결하게 수행합니다.
    """

    multimodal_system_prompt = f"""
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신의 임무는 레스토랑 오너의 질문에 대해 친절하고 상냥하게 답변하는 것입니다.
    
    아래의 [조건]에 맞게 [탬플릿] 형식으로 JSON 답변을 생성해주세요:
    '''json 같은 마크다운은 절대 포함하면 안됩니다.

    [조건]
    1. pinky_id는 "X번 핑키 카메라 켜봐"에서 X 입니다. 이때 X가 없으면 pinky_id는 비워두세요.
    2. pinky_task는 무조건 string형 "CAMERA_ON" 입니다.
    3. pinky_response는 아래 사용자 요청에 대한 string형 답변입니다.
    4. 출력은 반드시 JSON 형식의 텍스트만 반환해야 합니다.
    5. 절대 마크다운 형식(예: ```json, ``` 또는 ''' 등)을 사용하지 마세요.
    6. JSON 객체만 출력하세요. 문자열 앞뒤 공백 외에는 아무것도 포함하지 마세요.

    [탬플릿] : 반드시 큰따옴표로 감싸서 문자열 형태로 출력해야 합니다. 반드시 문자열 이외의 형식은 출력하지 않습니다. '''json 같은 마크다운은 절대 포함하면 안됩니다.
    {{
        "pinky_id": "",
        "pinky_task": "CAMREA_ON",
        "pinky_response": "",
    }}

    [사용자 요청]
    {user_prompt}
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

    try:
        alba_camera_on_response = json.loads(alba_camera_on_response)
    except json.JSONDecodeError:
        raise ValueError(f"Invalid JSON format: {alba_camera_on_response}")        

    return alba_camera_on_response

def discriminate_obstacle(detected_object_list) :
    """
    검출된 장애물이 동적 장애물인지, 정적 장애물인지 판별하여 json으로 반환해주는 함수입니다.

    Returns : JSON outputs
        {{
            "type": "dynamic / static"
        }}
    """

    if not detected_object_list:
        return {"type": ""}  # 아무것도 없으면 빈 문자열

    for obj in detected_object_list:
        if obj in dynamic_object_list:
            return {"type": "dynamic"}  # 하나라도 dynamic이면 dynamic

    return {"type": "static"}  # 모두 static일 경우

def save_alba_response(task, alba_response) :
    """
    AlbaGPT가 내놓은 json형 응답을 response 디렉토리에 json 파일로 저장하는 함수입니다. 
    """

    alba_response_dir = './contents/response'

    if not os.path.exists(alba_response_dir) :
        os.makedirs(alba_response_dir)

    if task == "GREETINGS" :
        greetings_file_path = os.path.join(alba_response_dir, 'GREETINGS' + '_response.json')

        with open(greetings_file_path, 'a', encoding="UTF-8") as json_writer:
            json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
            json_writer.write('\n\n')
            print(f"⭕️ Alba response successfully saved to {greetings_file_path}")
        
        json_writer.close()

    elif task == "CAMERA_ON" :
        CAMREA_ON_file_path = os.path.join(alba_response_dir, 'CAMERA_ON' + '_response.json')

        with open(CAMREA_ON_file_path, 'a', encoding="UTF-8") as json_writer:
            json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
            json_writer.write('\n\n')
            print(f"⭕️ Alba response successfully saved to {CAMREA_ON_file_path}")
        
        json_writer.close()
    
    else :
        maintenance_file_path = os.path.join(alba_response_dir, 'MAINTENANCE' + '_response.json')

        with open(maintenance_file_path, 'a', encoding="UTF-8") as json_writer:
            json.dump(alba_response, json_writer, indent=4, ensure_ascii=False)
            json_writer.write('\n\n')
            print(f"⭕️ Alba response successfully saved to {maintenance_file_path}")
        
        json_writer.close()        
    return