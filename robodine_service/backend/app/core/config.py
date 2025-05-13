import os

class Settings:
    PROJECT_NAME: str = "RoboDine"
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql+psycopg2://user:password@localhost:5432/robodine_db")

settings = Settings()


core = os.path.dirname(os.path.dirname(__file__))
app = os.path.dirname(core)
# backend 디렉토리의 images 폴더 경로
IMAGES_DIR = os.path.join(app, "images")
# 상위 디렉토리의 images 폴더가 존재하지 않으면 생성
if not os.path.exists(IMAGES_DIR):
    os.makedirs(IMAGES_DIR)

# backend 디렉토리의 videos 폴더 경로
VIDEOS_DIR = os.path.join(app, "videos")
# 상위 디렉토리의 videos 폴더가 존재하지 않으면 생성
if not os.path.exists(VIDEOS_DIR):
    os.makedirs(VIDEOS_DIR)