from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from dotenv import load_dotenv
from albagpt_server.config import alba_task_type_list

import re
import logging
import csv
import os

load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env"))

# 로거 생성
logger = logging.getLogger('alba_function')
logger.setLevel(logging.INFO)

# 핸들러 생성 (파일에 기록)
file_handler = logging.FileHandler('./contents/log/alba_function.log')
formatter = logging.Formatter('[%(asctime)s] %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)

# 로거에 핸들러 추가
logger.addHandler(file_handler)

def alba_task_discriminator(user_query, llm):
    """
    알바봇에게 입력된 명령의 종류를 GREETINGS, GOODBYE, CELEBRATE, none 중 하나로 구분합니다.
    
    Returns : 
        GREETINGS, GOODBYE, CELEBRATE, none 중 하나
    """
    
    discriminator_prompt = """
    당신은 레스토랑에서 업무를 하는 '핑키'라는 이름을 가진 모바일 로봇입니다.
    당신은 레스토랑에 온 손님의 다음 요청을 보고, 그것이 GREETINGS인지, GOODBYE인지, CELEBRATE인지, none인지 구분해야 합니다.

    다음 요청: {user_query}
    
    판단 기준은 다음과 같습니다:

    1. 요청이 인사인 경우 → 정확히 **"GREETINGS"** 라고만 출력하세요.
    2. 요청이 작별 인사인 경우 -> 정확히 **"GOODBYE"** 라고만 출력하세요.
    3. 요청에서 생일에 대한 내용이 있는 경우 -> 정확히 **"CELEBRATE"** 라고만 출력하세요.
    4. 그 외의 경우 -> 정확히 **"none"** 이라고 출력하세요.

    이때 아래 조건을 반드시 따르세요:
    - 출력은 반드시 **대문자** 단어 하나로만 답변하세요.
    - 추가 설명, 문장, 다른 단어는 절대 포함하지 마세요.
    - 가능한 한 명확하고 단정적으로 판단하세요.
    - 설명, 문장, 구두점은 답변으로 내놓지 마세요.
    """

    discriminator_template = PromptTemplate(
        template=discriminator_prompt,
        input_variables=["user_query"],
    )

    chain = LLMChain(
        llm=llm,
        prompt=discriminator_template
    )

    try:
        result = chain.invoke({
            "user_query": user_query,
        })

        discriminated_task = result.get('text', '').replace(".", "").strip()

        if discriminated_task == "":
            logger.warning("⚠️ LLM returned empty text.")
            return "none"
    except Exception as e:
        logger.error(f"❌ Error during LLMChain.invoke(): {e}")
        return "none"
    
    return discriminated_task

def extract_robot_id(user_query):
    """
    유저의 입력에서 해당하는 핑키의 ID를 반환해주는 함수입니다.
    
    Returns :
        robot_id가 정수 : (int)robot_id
        robot_id가 없는 경우 : None
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

def validate_alba_task_discriminator(user_query, llm):
    """
    alba_task_discriminator 함수로 구분한 task가 alba_task_type_list 배열의 원소 중 하나가 맞는지 한 번 더 판별해주는 함수입니다.

    Returns : 
        alba_task_type_list에 있는 경우 : {discriminated_task}
        alba_task_type_list에 없는 경우 : None
    """
    discriminated_task = alba_task_discriminator(user_query, llm)
    
    if discriminated_task not in alba_task_type_list :
        return None
    else :
        return discriminated_task
    
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
