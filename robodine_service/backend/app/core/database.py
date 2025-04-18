from contextlib import contextmanager
from sqlmodel import Session
from app.core.db_config import engine

@contextmanager
def get_session():
    session = Session(engine)
    try:
        yield session
    finally:
        session.close()
