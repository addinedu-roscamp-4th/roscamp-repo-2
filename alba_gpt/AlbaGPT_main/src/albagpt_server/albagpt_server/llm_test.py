from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain.chains import LLMChain
from dotenv import load_dotenv

import csv

load_dotenv()

user_query = "3번 핑키 사진 찍어"
alba_task_type_list = ['GREETINGS', 'TAKE_PICTURE', 'MAINTENANCE']

task_example_csv_path = '/home/addinedu/Desktop/roscamp-repo-2/alba_gpt/AlbaGPT_main/contents/example/task_example.csv' # 유저 프롬프트에 따른 분류된 task를 정리한 csv 파일
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
5. 판단이 애매할 경우에도 가능한 한 "none" 대신 GREETINGS, MAINTENANCE, TAKE_PICTURE 중 하나를 택하세요.


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
    "alba_task_type_list": alba_task_type_list,
    "task_example": task_example
})['text']

discriminated_task = discriminated_task.replace(".", "").strip()

print(discriminated_task)