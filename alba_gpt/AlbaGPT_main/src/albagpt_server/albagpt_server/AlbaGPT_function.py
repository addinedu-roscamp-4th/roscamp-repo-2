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

    logger.info(user_query)
    
    discriminator_prompt = """
    당신은 레스토랑에서 업무를 수행하는 '핑키'라는 모바일 로봇입니다.
    당신은 손님의 발화를 보고, 그것이 다음 중 어떤 유형에 속하는지 판단해야 합니다:

    - GREETINGS: 인사 표현 (예: "안녕", "좋은 아침", "하이", "오랜만이야" 등)
    - GOODBYE: 작별 인사 표현 (예: "잘 가", "또 봐", "안녕히 계세요", "잘 있어" 등)
    - CELEBRATE: 생일과 관련된 표현 (예: "오늘 내 생일이야", "생일 축하해줘", "오늘은 특별한 날", "나를 축하해줘", "생일파티" 등)
    - none: 위의 어느 것도 아닌 경우. 가급적이면 반환하지 마세요.

    예시:
    {task_example}

    아래 발화를 보고, GREETINGS / GOODBYE / CELEBRATE / none 중 **정확히 한 단어로만 출력**하세요.
    다른 말은 절대 하지 마세요. 설명도 하지 마세요. 마침표나 문장 부호도 붙이지 마세요.

    손님 발화:
    {user_query}

    정답:
    """

    discriminator_template = PromptTemplate(
        template=discriminator_prompt,
        input_variables=["user_query", "task_example"],
    )

    chain = LLMChain(
        llm=llm,
        prompt=discriminator_template
    )

    try:
        result = chain.invoke({
            "user_query": user_query,
            "task_example": task_example,
        })

        discriminated_task = result.get('text', '').replace(".", "").strip()

        logger.info(discriminated_task)

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
