from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from sqlmodel import Session, select
from datetime import datetime, timedelta
from typing import Optional
from passlib.context import CryptContext
from jose import JWTError, jwt
from os import getenv
from pydantic import BaseModel
import logging

from app.core.db_config import get_db, get_session
from app.models.user import User, UserCreate, UserRead, Token
from app.models.enums import UserRole

# Configure logging
logger = logging.getLogger("robodine.auth")
logger.setLevel(logging.INFO)

# Configuration
SECRET_KEY = getenv("SECRET_KEY", "your-secret-key")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# OAuth2 scheme
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="auth/login")

# 비밀번호 해싱
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

router = APIRouter()

# --- Auth Models ---
class LoginRequest(BaseModel):
    username: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    username: str
    role: Optional[str] = None

# --- Helper Functions ---
def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)

def authenticate_user(username: str, password: str, session: Session):
    logger.info(f"Authenticating user: {username}")
    user = session.query(User).filter(User.username == username).first()
    if not user:
        logger.warning(f"User not found: {username}")
        return False
    if not verify_password(password, user.password):
        logger.warning(f"Invalid password for user: {username}")
        return False
    logger.info(f"Authentication successful for user: {username}")
    return user

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Invalid authentication credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
        token_data = TokenData(username=username, role=payload.get("role"))
    except JWTError:
        logger.error("Invalid JWT token")
        raise credentials_exception
    user = db.query(User).filter(User.username == token_data.username).first()
    if user is None:
        logger.error(f"User from token not found: {token_data.username}")
        raise credentials_exception
    return user

# --- Router Endpoints ---
@router.post("/register", response_model=UserRead)
def register_user(user_data: UserCreate, session: Session = Depends(get_session)):
    logger.info(f"Registration attempt for username: {user_data.username}")
    
    # Check if username already exists
    existing_user = session.query(User).filter(User.username == user_data.username).first()
    if existing_user:
        logger.warning(f"Registration failed: Username already exists: {user_data.username}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already registered"
        )
    
    # Check if email already exists (only if email is provided)
    if user_data.email:
        existing_email = session.query(User).filter(User.email == user_data.email).first()
        if existing_email:
            logger.warning(f"Registration failed: Email already exists: {user_data.email}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )
    
    # Create new user with hashed password
    hashed_password = get_password_hash(user_data.password)
    new_user = User(
        username=user_data.username,
        email=user_data.email,
        full_name=user_data.full_name,
        role=user_data.role,
        password=hashed_password
    )
    
    # Add to database
    session.add(new_user)
    session.commit()
    session.refresh(new_user)
    
    logger.info(f"User registered successfully: {user_data.username}")
    return new_user

@router.post("/login", response_model=Token)
def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends(), session: Session = Depends(get_session)):
    logger.info(f"Login attempt for user: {form_data.username}")
    
    user = authenticate_user(form_data.username, form_data.password, session)
    if not user:
        logger.warning(f"Login failed for user: {form_data.username}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Update last login timestamp
    user.last_login = datetime.utcnow()
    session.commit()
    
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username, "user_id": user.id, "role": user.role}, 
        expires_delta=access_token_expires
    )
    
    logger.info(f"Login successful for user: {form_data.username}")
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/logout")
def logout():
    logger.info("Logout endpoint called")
    # Since JWT tokens are stateless, server-side logout isn't necessary
    # In a more complex system, you might implement token blacklisting here
    return {"status": "success", "message": "로그아웃 완료."} 