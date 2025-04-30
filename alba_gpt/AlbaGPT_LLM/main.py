import Alba_LLM_templates
import os
import sys

from multiprocessing import Process, Event, Manager
from langchain.memory import ConversationBufferMemory
from Alba_LLM_templates import alba_work_type_list
from Alba_LLM_templates import save_alba_response

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

def main_thread(shared_data) :
    memory = ConversationBufferMemory(memory_key="chat_history", input_key="user_query")
    chat_history = memory.load_memory_variables({})["chat_history"]

    user_query_example_list = ["1번 핑키 안녕!", "오늘 기분이 어때?, 1번 핑키?","3 테이블 청소해, 4번 핑키!", "핑키들, 지금 매장에 도둑이 들었어!", "2번 핑키 4번 테이블이 더럽잖아.", "5번 테이블에 셀러드 하나, 2번 핑키!", "핑키들 5번 테이블 손님 오늘 생일이래", "2번 핑키 카메라 켜봐"]

    for user_query in user_query_example_list :
        discriminated_task = Alba_LLM_templates.validate_alba_task_discriminator(user_query)

        if discriminated_task == "greetings" :
            alba_response = Alba_LLM_templates.generate_alba_greetings_response(user_query, chat_history, memory)
            save_alba_response("greetings", alba_response)

        elif discriminated_task == "camera on" :
            alba_response = Alba_LLM_templates.generate_alba_camera_on_response(user_query, chat_history, shared_data.latest_frame)
            save_alba_response("camera on", alba_response)

        else :
            for work in alba_work_type_list :
                if discriminated_task == work :
                    alba_response = Alba_LLM_templates.generate_alba_work_response(user_query, chat_history, memory, work)
                    save_alba_response(discriminated_task, alba_response)

if __name__=="__main__":
    stop_event = Event()
    manager = Manager()
    
    shared_data = manager.Namespace()
    shared_data.latest_frame = None

    udp_process = Process(target=AlbaGPT_UDP.alba_udp_server, args=(stop_event,shared_data))
    main_process = Process(target=main_thread, args=(shared_data,))

    udp_process.start()
    main_process.start()

    udp_process.join()
    main_process.join()