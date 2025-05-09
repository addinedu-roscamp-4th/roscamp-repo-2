#robodine_service/backend/app/routes/robot.py

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field as PydanticField

from app.core.db_config import get_db
from app.models import Robot, RobotCommand, Pose6D
from app.models.enums import EntityType, CommandStatus, RobotStatus
from app.routes.events import log_info, log_warning, log_error

router = APIRouter()

# --- Robot Models ---
class RobotRegisterRequest(BaseModel):
    robot_id: int
    robot_type: EntityType
    mac_address: str
    ip_address: str

class RobotResponse(BaseModel):
    robot_id: int
    robot_type: EntityType
    mac_address: str
    ip_address: str
    timestamp: datetime

class CommandRequest(BaseModel):
    robot_id: int
    command: str
    parameters: dict
    status: Optional[CommandStatus] = CommandStatus.PENDING

class CommandResponse(BaseModel):
    id: int
    status: str = "success"
    message: str = "명령이 성공적으로 전송되었습니다."

class CommandStatusRequest(BaseModel):
    id: int
    robot_id: int
    command: str
    parameters: dict
    status: CommandStatus

class CommandStatusResponse(BaseModel):
    status: str
    command: str
    parameters: dict
    message: str

class CommandListResponse(BaseModel):
    id: int
    robot_id: int
    command: str
    parameters: dict
    status: str
    timestamp: datetime
    executed_at: Optional[datetime] = None

# --- Router Endpoints ---
@router.get("/commands", response_model=List[CommandListResponse])
def get_all_commands(db: Session = Depends(get_db)):
    # Get all commands
    commands = (
        db.query(RobotCommand)
        .order_by(RobotCommand.id.desc())
        .all()
    )
    if not commands:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="No commands found"
        )
    
    return [
        CommandListResponse(
            id=cmd.id,
            robot_id=cmd.robot_id,
            command=cmd.command,
            parameters=cmd.parameters,
            status=cmd.status,
            timestamp=cmd.timestamp,
            executed_at=cmd.executed_at
        ) for cmd in commands
    ]

@router.post("/register", response_model=dict)
def register_robot(robot_data: RobotRegisterRequest, db: Session = Depends(get_db)):
    # Check if robot already exists
    existing_robot = db.query(Robot).filter(Robot.robot_id == str(robot_data.robot_id)).first()
    
    if existing_robot:
        # Update existing robot
        existing_robot.type = robot_data.robot_type
        existing_robot.mac_address = robot_data.mac_address
        existing_robot.ip_address = robot_data.ip_address
        existing_robot.timestamp = datetime.utcnow()
        db.add(existing_robot)
    else:
        # Create new robot
        new_robot = Robot(
            robot_id=str(robot_data.robot_id),
            type=robot_data.robot_type,
            mac_address=robot_data.mac_address,
            ip_address=robot_data.ip_address,
            timestamp=datetime.utcnow()
        )
        db.add(new_robot)
    
    db.commit()
    
    return {
        "robot_id": robot_data.robot_id,
        "status": "success",
        "message": "로봇 정보가 성공적으로 등록되었습니다."
    }

@router.get("", response_model=List[RobotResponse])
def get_robots(db: Session = Depends(get_db)):
    robots = db.query(Robot).all()
    return [
        RobotResponse(
            robot_id=int(robot.robot_id),
            robot_type=robot.type,
            mac_address=robot.mac_address,
            ip_address=robot.ip_address,
            timestamp=robot.timestamp
        ) for robot in robots
    ]

@router.get("/{robot_id}", response_model=RobotResponse)
def get_robot(robot_id: int, db: Session = Depends(get_db)):
    robot = db.query(Robot).filter(Robot.robot_id == str(robot_id)).first()
    
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    return RobotResponse(
        robot_id=int(robot.robot_id),
        robot_type=robot.type,
        mac_address=robot.mac_address,
        ip_address=robot.ip_address,
        timestamp=robot.timestamp
    )

@router.put("/{robot_id}", response_model=RobotResponse)
def update_robot(robot_id: int, robot_data: RobotRegisterRequest, db: Session = Depends(get_db)):
    robot = db.query(Robot).filter(Robot.robot_id == str(robot_id)).first()
    
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    # Update robot details
    robot.type = robot_data.robot_type
    robot.mac_address = robot_data.mac_address
    robot.ip_address = robot_data.ip_address
    robot.timestamp = datetime.utcnow()
    
    db.add(robot)
    db.commit()
    
    return RobotResponse(
        robot_id=int(robot.robot_id),
        robot_type=robot.type,
        mac_address=robot.mac_address,
        ip_address=robot.ip_address,
        timestamp=robot.timestamp
    )

@router.delete("/{robot_id}", response_model=dict)
def delete_robot(robot_id: int, db: Session = Depends(get_db)):
    robot = db.query(Robot).filter(Robot.robot_id == str(robot_id)).first()
    
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    db.delete(robot)
    db.commit()
    
    return {
        "status": "success",
        "message": "로봇 정보가 성공적으로 삭제되었습니다."
    }


@router.post("/commands/{robot_id}/command", response_model=CommandResponse)
def send_command(robot_id: int, command_data: CommandRequest, db: Session = Depends(get_db)):
    """로봇에 명령 전송"""
    # 로봇 존재 여부 확인
    robot = db.query(Robot).filter(Robot.robot_id == str(robot_id)).first()
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    # 명령 파라미터 처리
    parameters = command_data.parameters
    
    # 새 명령 생성
    new_command = RobotCommand(
        robot_id=robot.id,
        command=command_data.command,
        parameters=parameters,
        status=command_data.status,
        timestamp=datetime.utcnow()
    )
    
    db.add(new_command)
    db.commit()
    db.refresh(new_command)
    
    return CommandResponse(
        id=new_command.id
    )

@router.put("/commands/{command_id}/status", response_model=CommandStatusResponse)
def update_command_status(command_id: int, status_data: CommandStatusRequest, db: Session = Depends(get_db)):
    # Find command
    command = db.query(RobotCommand).filter(RobotCommand.id == command_id).first()
    if not command:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Command with ID {command_id} not found"
        )
    
    # Update status
    command.status = status_data.status
    if status_data.status == CommandStatus.EXECUTED:
        command.executed_at = datetime.utcnow()

    db.add(command)
    db.commit()
    
    return CommandStatusResponse(
        status="success",
        message="명령의 상태가 성공적으로 변경되었습니다."
    )


@router.get("/commands/{robot_id}", response_model=List[CommandListResponse])
def get_commands(robot_id: int, db: Session = Depends(get_db)):
    # Check if robot exists
    robot = db.query(Robot).filter(Robot.robot_id == str(robot_id)).first()
    if not robot:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Robot with ID {robot_id} not found"
        )
    
    # Get commands for this robot
    commands = db.query(RobotCommand).filter(RobotCommand.robot_id == robot.id).all()
    
    return [
        CommandListResponse(
            id=cmd.id,
            command=cmd.command,
            parameters=cmd.parameters,
            status=cmd.status
        ) for cmd in commands
    ]

@router.put("/commands/{command_id}", response_model=CommandResponse)
def update_command(command_id: int, command_data: CommandRequest, db: Session = Depends(get_db)):
    """기존 명령어 정보 수정"""
    try:
        print(f"명령어 수정 요청: ID={command_id}, 데이터={command_data}")
        
        # 명령어 존재 여부 확인
        command = db.query(RobotCommand).filter(RobotCommand.id == command_id).first()
        if not command:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Command with ID {command_id} not found"
            )
        
        # 로봇 존재 여부 확인
        robot = db.query(Robot).filter(Robot.robot_id == str(command_data.robot_id)).first()
        if not robot:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Robot with ID {command_data.robot_id} not found"
            )
        
        # 유효한 상태 값인지 확인
        status_value = command_data.status
        if not isinstance(status_value, CommandStatus):
            try:
                # 문자열로 전달된 경우 Enum으로 변환
                status_value = CommandStatus(status_value)
            except ValueError:
                valid_statuses = ", ".join([s.value for s in CommandStatus])
                raise HTTPException(
                    status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                    detail=f"Invalid status value. Must be one of: {valid_statuses}"
                )
        
        # 명령어 정보 업데이트
        command.robot_id = robot.id
        command.command = command_data.command
        command.parameters = command_data.parameters
        command.status = status_value
        
        # 상태가 EXECUTED로 변경되면 executed_at 설정
        if status_value == CommandStatus.EXECUTED and not command.executed_at:
            command.executed_at = datetime.utcnow()
        
        db.add(command)
        db.commit()
        db.refresh(command)
        
        print(f"명령어 수정 성공: ID={command_id}")
        
        return CommandResponse(
            id=command.id
        )
    except HTTPException:
        # 이미 정의된 HTTP 예외는 그대로 전달
        raise
    except Exception as e:
        # 기타 오류는 서버 오류로 처리하고 상세 정보 제공
        error_msg = str(e)
        print(f"명령어 수정 중 오류 발생: {error_msg}")
        
        # SQL Alchemy 에러 메시지에서 상태 값 오류 감지
        if "invalid input value for enum commandstatus" in error_msg:
            valid_statuses = ", ".join([s.value for s in CommandStatus])
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail=f"유효하지 않은 상태 값입니다. 유효한 값: {valid_statuses}"
            )
            
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"명령어 수정 중 오류 발생: {error_msg}"
        ) 