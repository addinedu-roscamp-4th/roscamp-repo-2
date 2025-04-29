from langchain.memory import ConversationBufferMemory
import Alba_LLM_templates
import os
import sys
from multiprocessing import Process, Event, Manager

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from AlbaGPT_communication import AlbaGPT_UDP

def main_thread(shared_data) :
    alba_work_type_list = ["cleaning", "serving", "birthday", "emergency"]
    memory = ConversationBufferMemory(memory_key="chat_history", input_key="user_query")
    chat_history = memory.load_memory_variables({})["chat_history"]

    user_query_list = ["오늘도 최고의 하루가 될거야!","3 테이블 청소해, 4번 핑키!", "핑키들, 지금 매장에 화재 발생!", "1번 핑키 2번 테이블이 더럽잖아.", "5번 테이블에 셀러드 하나, 2번 핑키!", "3번 핑키 카메라 켜봐"]

    image_path = '/home/addinedu/roscamp-repo-2/alba_gpt/AlbaGPT_LLM_ws/test_image' # 현재는 임의로 설정

    for user_query in user_query_list :
        discriminated_task = Alba_LLM_templates.validate_alba_task_discriminator(user_query)

        if discriminated_task == "greetings" :
            Alba_LLM_templates.generate_alba_greetings_response(user_query, chat_history, memory)

        elif discriminated_task == "camera on" :
            Alba_LLM_templates.generate_alba_camera_on_response(user_query, chat_history, shared_data.latest_frame)

        else :
            for work in alba_work_type_list :
                if discriminated_task == work :
                    Alba_LLM_templates.generate_alba_work_response(user_query, chat_history, memory, work)

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