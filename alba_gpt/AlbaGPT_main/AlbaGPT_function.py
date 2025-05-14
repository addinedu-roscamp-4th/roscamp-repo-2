from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain_teddynote.models import MultiModal
from dotenv import load_dotenv
from AlbaGPT_main import alba_task_type_list, dynamic_object_list

import re
import csv
import cv2
import os
import numpy as np
import uuid
import time
import logging

load_dotenv()

# 로거 생성
logger = logging.getLogger('alba_function')
logger.setLevel(logging.INFO)

# 핸들러 생성 (파일에 기록)
file_handler = logging.FileHandler('./contents/log/alba_function.log')
formatter = logging.Formatter('[%(asctime)s] %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)

# 로거에 핸들러 추가
logger.addHandler(file_handler)

def alba_task_discriminator(user_query, memory, alba_task_type_list=alba_task_type_list):
    """
    알바봇에게 입력된 프롬프트가 어떤 type의 명령인지 구분해주는 함수입니다.
    
    Returns : 
        MAINTENANCE, GREETINGS, TAKE_PICTURE, none 중 하나
    """

    chat_history = []

    chat_history = "\n".join(
    f"user_query: {memory['messages'][idx]['question']}, response: {memory['messages'][idx]['answer']}"
    for idx in range(len(memory))
    if memory['messages'][idx]['answer'] not in ["채팅 서버에 연결할 수 없습니다. 잠시 후 다시 시도해주세요.", "None"]
    )

    logger.info(chat_history)

    task_example_csv_path = './contents/example/task_example.csv' # 유저 프롬프트에 따른 분류된 task를 정리한 csv 파일
    task_example_list = []

    with open(task_example_csv_path, 'r', encoding='utf-8') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)
    
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

    당신은 오너의 다음 요청을 보고, 그것이 MAINTENANCE인지, GREETINGS인지, TAKE_PICTURE인지, none인지 구분해야 합니다.

    다음 요청: {user_query}
    
    판단 기준은 다음과 같습니다:

    1. 요청이 인사인 경우 → 정확히 **"GREETINGS"** 라고만 출력하세요.
    2. 요청이 작업 명령일 경우 → 해당 작업이 {alba_task_type_list} 배열 원소 중 무엇인지 판단하고, 해당 태스크 이름만 출력하세요.
    3. 만약 "카메라 켜봐", "무엇이 보이냐", "보이는 것을 설명해달라", "뭐 때문에 못 가고 있어?..등등과 같은 요청이 포함된다면, **TAKE_PICTURE**으로 간주하세요. 이 경우 '카메라'라는 단어가 없어도 무조건 TAKE_PICTURE으로 판단하세요.
    4. {alba_task_type_list}의 배열에 없는 원소에 대해서는 **"none"**을 출력하세요.
    5. "X번 핑키도", "1번은?", "걔는?"과 같이 이전 발화를 전제로 한 문장은 반드시 직전 user_query인 "{chat_history}"를 고려해 판단하세요.
    예를 들어 "{chat_history}"이 GREETINGS로 분류되었다면, 유사하거나 동일한 요청이라고 간주해서 대답하세요.
    예를 들어 "{chat_history}"이 MAINTENANCE로 분류되었다면, 유사하거나 동일한 요청이라고 간주해서 대답하세요.
    예를 들어 "{chat_history}"이 TAKE_PICTURE로 분류되었다면, 유사하거나 동일한 요청이라고 간주해서 대답하세요.
    6. 판단이 애매할 경우에도 가능한 한 "none" 대신 GREETINGS, MAINTENANCE, TAKE_PICTURE 중 하나를 택하세요.


    이때 아래 조건을 반드시 따르세요:
    - 출력은 반드시 **대문자** 단어 하나로만 답변하세요.
    - 추가 설명, 문장, 다른 단어는 절대 포함하지 마세요.
    - 가능한 한 명확하고 단정적으로 판단하세요.
    - 설명, 문장, 구두점은 답변으로 내놓지 않아요.
    """

    discriminator_template = PromptTemplate(
        template=discriminator_prompt,
        input_variables=["user_query", "chat_history", "alba_task_type_list", "task_example"],
    )

    # 객체 생성
    llm = ChatOpenAI(
        temperature=0.7,  # 창의성 (0.0 ~ 2.0)
        max_tokens=2048,  # 최대 토큰수
        model_name="gpt-4o",  # 모델명
    )

    chain = LLMChain(
        llm=llm,
        prompt=discriminator_template
    )

    discriminated_task = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "alba_task_type_list": alba_task_type_list,
        "task_example": task_example
    })['text']

    discriminated_task = discriminated_task.replace(".", "").strip()
    
    return discriminated_task

def validate_alba_task_discriminator(user_query, memory, alba_task_type_list=alba_task_type_list):
    """
    alba_task_discriminator 함수로 구분한 task가 alba_task_type_list 배열의 원소 중 하나가 맞는 지 한 번 더 판별해주는 함수입니다.

    Returns : 
        alba_task_type_list에 있는 경우 : {discriminated_task}
        alba_task_type_list에 없는 경우 : None
    """
    discriminated_task = alba_task_discriminator(user_query, memory)
    
    if discriminated_task not in alba_task_type_list :
        return None
    else :
        return discriminated_task

def extract_robot_id(user_query):
    """
    user_query에서 "X번 핑키"의 X를 추출해 반환해주는 함수입니다.
    
    Returns :
        robot_id (int) or 0 if not found
    """
    # 정규표현식 패턴: 숫자 + "번 핑키"
    match = re.search(r'(\d+)번 핑키', user_query)
    
    if match:
        # 숫자 부분만 추출하여 정수형으로 변환
        robot_id = int(match.group(1))
        logger.info(f"🪪 Discriminated Robot ID: {robot_id}")
        return robot_id
    else:
        # 패턴이 없으면 0 반환
        logger.info("🪪 No matching Robot ID found. Returning 0.")
        return 0
    
def validate_discriminate_robot_id(robot_id_response) :
    """
    extract_robot_id 함수가 robot_id를 잘 추출해 냈는지 한 번 더 판별해주는 함수입니다.
    
    Returns :
        robot_id가 정수 : (int)robot_id
        robot_id가 없는 경우 : None
    """    
    if robot_id_response.isdigit():
        return int(robot_id_response)
    else:
        return 0

def generate_alba_greetings_response(user_query, memory):
    """
    alba_task_discriminator 함수로 구분된 task가 'GREETINGS'인 경우 user_query에 대해 대답을 생성해주는 함수입니다.
    
    Returns : alba_greetings_response (str)
    """
    greetings_example_path = './contents/example/greetings_example.txt' # GREETINGS 상황에 대한 응답을 정리한 텍스트 파일

    with open(greetings_example_path, 'r', encoding='utf-8') as greetings_example_file:
        # 파일 내용을 한 번에 읽고 strip()으로 양쪽 공백 제거
        greetings_example = greetings_example_file.read().strip()

    chat_history = "\n".join(
    f"user_query: {memory['messages'][idx]['question']}, response: {memory['messages'][idx]['answer']}"
    for idx in range(len(memory))
    if memory['messages'][idx]['answer'] not in ["채팅 서버에 연결할 수 없습니다. 잠시 후 다시 시도해주세요.", "None"]
    )

    alba_greetings_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신은 레스토랑 오너의 {user_query}에 대해 친절하고 상냥하게, 1~3줄 이내로 너무 길지 않게 답변하여야 합니다.    

    [조건]
    1. response_text는 {user_query}에 대한 string형 답변입니다.
    2. {greetings_example} 중 {chat_history}에 있는 response와는 다른 답변으로 랜덤하게 하나를 선택해 반환해주세요.
    3. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.
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
    )

    alba_greetings_response = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "greetings_example": greetings_example,
    })['text']

    return alba_greetings_response

def generate_alba_none_response(user_query):
    """
    alba_task_discriminator 함수로 구분된 task가 'None'인 경우 user_query에 대해 대답을 생성하여 전달해주는 함수입니다.

    Returns : alba_none_response (str)
    """
    
    alba_none_response = f"제가 이해할 수 없었어요 😭 다른 질문 있으신가요?"
    
    return alba_none_response

def generate_alba_maintenance_response(user_query, memory):
    """
    alba_task_discriminator 함수로 구분된 task가 'MAINTENANCE'인 경우 user_query에 대해 대답을 생성해주는 함수입니다.
    
    Returns : alba_maintenance_response (str)
    """
    maintenance_example_path = './contents/example/maintenance_example.txt' # maintenance 상황에 대한 응답을 정리한 텍스트 파일

    with open(maintenance_example_path, 'r', encoding='utf-8') as maintenance_example_file:
        # 파일 내용을 한 번에 읽고 strip()으로 양쪽 공백 제거
        maintenance_example = maintenance_example_file.read().strip()

    chat_history = "\n".join(
    f"user_query: {memory['messages'][idx]['question']}, response: {memory['messages'][idx]['answer']}"
    for idx in range(len(memory))
    if memory['messages'][idx]['answer'] not in ["채팅 서버에 연결할 수 없습니다. 잠시 후 다시 시도해주세요.", "None"]
    )

    alba_maintenance_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
    지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

    당신은 레스토랑 오너의 {user_query}에 대해 {chat_history}와 연관이 있다면 참고하여 1~3줄 이내로 너무 길지 않게 답변하여야 합니다.    

    [조건]
    1. response_text는 {user_query}에 대한 string형 답변입니다.
    2. {maintenance_example} 중 {chat_history}에 있는 response와는 다른 답변으로 랜덤하게 하나를 선택해 반환해주세요.
    3. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.
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
        verbose=True
    )

    alba_maintenance_response = chain.invoke({
        "user_query": user_query,
        "chat_history": chat_history,
        "maintenance_example" : maintenance_example
    })['text']

    print(alba_maintenance_response)

    return alba_maintenance_response

def detect_object_obstacles(detected_objects):
    """
    감지된 장애물 중 동적 장애물만 추출해주는 함수입니다.

    Returns :
        detected_objet_list (list)
    """
    detected_object_list = []

    for detected_object in detected_objects :
        detected = detected_object.categories[0].category_name
        if detected in dynamic_object_list :
            detected_object_list.append(detected)

    return detected_object_list

def generate_alba_take_picture_response(user_query, memory, shared_dict, robot_id):
    """
    alba_task_discriminator 함수로 구분된 task가 'TAKE_PICTURE'인 경우 user_query에 대해 해당하는 알바 봇으로부터 수신되는 영상의 이미지 프레임을 토대로 상황을 해석해 반환해주는 함수입니다.

    Returns : alba_take_picture_response (str), image_path(str or None)
    """
    image_dir = './contents/image'

    if not os.path.exists(image_dir) :
        os.makedirs(image_dir)

    image_path = os.path.join(image_dir, str(uuid.uuid4().hex) + '_' + time.strftime('%Y-%m-%d %H-%M-%S') + '.jpg')

    shared_id_dict = shared_dict.get(robot_id)
    
    if shared_id_dict :
        shared_id_dict = shared_dict[robot_id]
        
        ip_port = shared_id_dict["ip_port"]
        shared_sub_dict = shared_dict[ip_port]
        encoded_latest_frame = shared_sub_dict["latest_frame"]
        detected_objects = shared_sub_dict["detected_object"]
        
        detected_object_list = detect_object_obstacles(detected_objects)
        logger.info(f"💡 Detected Objects : {detected_object_list}")
            
        np_data = np.frombuffer(encoded_latest_frame, dtype=np.uint8)
        decoded_detected_image = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        if decoded_detected_image is None:
            raise ValueError("❌ Failed to decode image from decoded_detected_image")

        cv2.imwrite(image_path, decoded_detected_image)
        logger.info(f"📷 Image successfully saved to {image_path}")

        object_info = ', '.join(detected_object_list)

        if not object_info: # 감지된 동적 장애물이 없다면
            alba_take_picture_response = "아무 문제 없이 임무를 잘 수행하고 있어요 🤗"
        else :
            obstacle_example_path = './contents/example/obstacle_example.txt' # TAKE PICTURES 상황에 대한 응답을 정리한 텍스트 파일
            with open(obstacle_example_path, 'r', encoding='utf-8') as obstacle_example_file:
                # 파일 내용을 한 번에 읽고 strip()으로 양쪽 공백 제거
                obstacle_meetup_example = obstacle_example_file.read().strip()

            chat_history = "\n".join(
            f"user_query: {memory['messages'][idx]['question']}, response: {memory['messages'][idx]['answer']}"
            for idx in range(len(memory))
            if memory['messages'][idx]['answer'] not in ["채팅 서버에 연결할 수 없습니다. 잠시 후 다시 시도해주세요.", "None"]
            )

            user_prompt = f"""
            아래는 감지된 사물 목록입니다: {object_info}
            이 사물들은 경로를 가로막고 있습니다.

            당신의 임무는 다음과 같습니다:
            1. "{object_info} {obstacle_meetup_example}"와 같은 식으로 대답하여야 합니다. 이때 {object_info}는 한국어로 번역해주어 말을 해야 합니다.
            2. {object_info}가 '사람'인 경우에도 그냥 "{object_info} {obstacle_meetup_example}"와 같은 식으로 대답하기만 하면 됩니다.
            3. 중복되는 {object_info}에 대해서는 한 번만 대답합니다.
            4. 답변은 1-3문장 이내로 간결하게 수행합니다.
            """
            multimodal_system_prompt = f"""
            당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다
            지금까지 당신과 레스토랑 오너가 나눈 대화는 다음과 같습니다 : {chat_history}

            당신은 레스토랑 오너의 {user_query}에 대해 친절하고 상냥하게, 1~3줄 이내로 너무 길지 않게 답변하여야 합니다.   

            [조건]
            1. response_text는 {user_query}에 대한 string형 답변입니다.
            2. {obstacle_meetup_example} 중 {chat_history}에 있는 response와는 다른 답변으로 랜덤하게 하나를 선택해 반환해주세요.
            3. 위의 조건 이외에 다른 정수, 단어, 문장은 생성하지 않습니다.
            """

            # 객체 생성
            llm = ChatOpenAI(
                temperature=0.4,  # 창의성 (0.0 ~ 2.0)
                max_tokens=2048,  # 최대 토큰수
                model_name="gpt-4o",  # 모델명
            )

            multimodal_llm = MultiModal(llm, system_prompt=multimodal_system_prompt, user_prompt=user_prompt)
            alba_take_picture_response = multimodal_llm.invoke(image_path)
    else :
        logger.warning(f"🚫 [ERROR] : AlbaBot {robot_id} camera is not on")
        alba_take_picture_response = f"현재 알바봇 {robot_id}의 카메라가 꺼져 있어요 😢"
        image_path = None

    return alba_take_picture_response, image_path