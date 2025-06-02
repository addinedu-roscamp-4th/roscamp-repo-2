import rclpy
from rclpy.node import Node
from llm_srv.srv import Mysrv  # srv 파일 import

class LLMServiceServer(Node):

    def __init__(self):
        super().__init__('llm_service_server')
        self.srv = self.create_service(Mysrv, 'llm_response', self.handle_llm_request)
        self.get_logger().info("✅ LLM 서비스 서버가 시작되었습니다.")

    def handle_llm_request(self, request, response):
        self.get_logger().info(f"📨 요청 수신: robot_id={request.robot_id}, robot_task={request.robot_task}")

        # 응답 생성 로직 예시
        response_text = f"🔧 {request.robot_id}번 로봇에게 할당된 작업은 '{request.robot_task}'입니다."
        response.response_text = response_text

        self.get_logger().info(f"💬 응답 전송: {response_text}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 서비스 서버 종료 (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
