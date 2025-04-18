import os

class Settings:
    PROJECT_NAME: str = "RoboDine"
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql+psycopg2://user:password@localhost:5432/robodine_db")

settings = Settings()
