# robodine_service/backend/app/routes/streaming.py

import os
import time
import threading
import logging
import asyncio
import cv2
import uuid
from datetime import datetime, timedelta
from typing import Optional
from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from sqlalchemy import create_engine, select

from app.core.db_config import get_db
from app.models.video_stream import VideoStream
from app.models.enums import StreamSourceType, StreamStatus

logger = logging.getLogger("robodine.streaming_service")
router = APIRouter()

class VideoRecorder:
    """UDP 스트림을 주기적으로 녹화하는 클래스"""
    
    def __init__(self, udp_receiver, db_session_factory, recording_interval=300, fps=30):
        """
        Args:
            udp_receiver: UDP 수신기 인스턴스
            db_session_factory: 데이터베이스 세션 팩토리
            recording_interval: 녹화 간격 (초, 기본 5분)
            fps: 프레임 레이트 (기본 30fps)
        """
        self.udp_receiver = udp_receiver
        self.db_session_factory = db_session_factory
        self.recording_interval = recording_interval
        self.fps = fps
        self.is_recording = False
        self.recording_thread = None
        self.current_video_writer = None
        self.current_recording_path = None
        self.current_stream_id = None
        self.recording_start_time = None
        
        # 녹화 파일 저장 디렉토리 생성
        self.recordings_dir = os.path.join(os.getcwd(), "recordings")
        os.makedirs(self.recordings_dir, exist_ok=True)
        
    def start_recording(self):
        """녹화 시작"""
        if self.is_recording:
            logger.warning("이미 녹화가 진행 중입니다.")
            return
            
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._recording_loop, daemon=True)
        self.recording_thread.start()
        logger.info("비디오 녹화 시작됨")
        
    def stop_recording(self):
        """녹화 중지"""
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join(timeout=5.0)
        self._finalize_current_recording()
        logger.info("비디오 녹화 중지됨")
        
    def _recording_loop(self):
        """녹화 메인 루프"""
        while self.is_recording:
            try:
                # UDP 스트림이 활성화되어 있는지 확인
                if not self.udp_receiver.has_frames():
                    logger.debug("UDP 스트림에 프레임이 없어 녹화 대기 중...")
                    time.sleep(5)
                    continue
                
                # 새 녹화 세션 시작
                self._start_new_recording_session()
                
                # 지정된 시간 동안 녹화
                session_start_time = time.time()
                frame_count = 0
                
                while (self.is_recording and 
                       time.time() - session_start_time < self.recording_interval and
                       self.current_video_writer is not None):
                    
                    # UDP에서 프레임 가져오기
                    frame = self.udp_receiver.get_latest_frame()
                    if frame is not None:
                        # 프레임을 비디오 파일에 쓰기
                        self.current_video_writer.write(frame)
                        frame_count += 1
                        
                        # 로그 (매 300프레임마다)
                        if frame_count % 300 == 0:
                            logger.debug(f"녹화 중... 프레임: {frame_count}")
                    
                    # FPS에 맞춰 대기
                    time.sleep(1.0 / self.fps)
                
                # 현재 녹화 세션 종료
                self._finalize_current_recording()
                
                # 잠시 대기 후 다음 녹화 시작
                if self.is_recording:
                    time.sleep(1)
                    
            except Exception as e:
                logger.error(f"녹화 중 오류 발생: {e}")
                self._finalize_current_recording()
                time.sleep(5)  # 오류 발생 시 5초 대기
                
    def _start_new_recording_session(self):
        """새 녹화 세션 시작"""
        try:
            # 현재 진행 중인 녹화가 있으면 종료
            self._finalize_current_recording()
            
            # 새 파일명 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"udp_stream_{timestamp}.mp4"
            self.current_recording_path = os.path.join(self.recordings_dir, filename)
            
            # 비디오 라이터 생성
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.current_video_writer = cv2.VideoWriter(
                self.current_recording_path,
                fourcc,
                self.fps,
                (640, 480)  # 해상도 설정
            )
            
            if not self.current_video_writer.isOpened():
                logger.error(f"비디오 파일 생성 실패: {self.current_recording_path}")
                self.current_video_writer = None
                return
            
            self.recording_start_time = datetime.now()
            self.current_stream_id = str(uuid.uuid4())
            
            logger.info(f"새 녹화 세션 시작: {filename}")
            
        except Exception as e:
            logger.error(f"녹화 세션 시작 중 오류: {e}")
            self.current_video_writer = None
            
    def _finalize_current_recording(self):
        """현재 녹화 세션 종료 및 데이터베이스 저장"""
        if self.current_video_writer is None:
            return
            
        try:
            # 비디오 라이터 해제
            self.current_video_writer.release()
            
            # 파일이 실제로 생성되었는지 확인
            if os.path.exists(self.current_recording_path):
                file_size = os.path.getsize(self.current_recording_path)
                
                # 최소 크기 이상의 파일만 저장 (1KB 이상)
                if file_size > 1024:
                    # 데이터베이스에 저장
                    self._save_to_database()
                    logger.info(f"녹화 완료 및 데이터베이스 저장: {self.current_recording_path} ({file_size} bytes)")
                else:
                    # 크기가 너무 작으면 파일 삭제
                    os.remove(self.current_recording_path)
                    logger.warning(f"녹화 파일이 너무 작아 삭제됨: {self.current_recording_path}")
            else:
                logger.warning(f"녹화 파일이 생성되지 않음: {self.current_recording_path}")
                
        except Exception as e:
            logger.error(f"녹화 종료 중 오류: {e}")
        finally:
            # 현재 상태 초기화
            self.current_video_writer = None
            self.current_recording_path = None
            self.current_stream_id = None
            self.recording_start_time = None
            
    def _save_to_database(self):
        """녹화된 영상 정보를 데이터베이스에 저장"""
        try:
            # 데이터베이스 세션 생성
            db = next(self.db_session_factory())
            
            # VideoStream 객체 생성
            video_stream = VideoStream(
                source_type=StreamSourceType.WEBCAM,
                source_id="udp_stream_camera",
                last_checked=datetime.now(),
                recording_started_at=self.recording_start_time,
                recording_ended_at=datetime.now(),
                url=f"/recordings/{os.path.basename(self.current_recording_path)}",
                status=StreamStatus.INACTIVE,
                recording_path=self.current_recording_path
            )
            
            # 데이터베이스에 저장
            db.add(video_stream)
            db.commit()
            db.refresh(video_stream)
            
            logger.info(f"데이터베이스 저장 완료: Stream ID {video_stream.id}")
            
        except Exception as e:
            logger.error(f"데이터베이스 저장 중 오류: {e}")
            if db:
                db.rollback()
        finally:
            if db:
                db.close()

# 전역 비디오 레코더 인스턴스
video_recorder = None

def initialize_video_recorder(udp_receiver, db_session_factory):
    """비디오 레코더 초기화"""
    global video_recorder
    if video_recorder is None:
        video_recorder = VideoRecorder(
            udp_receiver=udp_receiver,
            db_session_factory=db_session_factory,
            recording_interval=300,  # 5분마다 녹화
            fps=30
        )
        video_recorder.start_recording()
        logger.info("비디오 레코더 초기화 및 시작됨")

def get_video_recorder():
    """비디오 레코더 인스턴스 반환"""
    return video_recorder

@router.get("/recordings")
async def get_recordings(db: Session = Depends(get_db)):
    """저장된 녹화 목록 조회"""
    try:
        recordings = db.query(VideoStream).filter(
            VideoStream.source_type == StreamSourceType.WEBCAM
        ).order_by(VideoStream.recording_started_at.desc()).all()
        
        recording_list = []
        for recording in recordings:
            recording_list.append({
                "id": recording.id,
                "source_id": recording.source_id,
                "recording_started_at": recording.recording_started_at.isoformat() if recording.recording_started_at else None,
                "recording_ended_at": recording.recording_ended_at.isoformat() if recording.recording_ended_at else None,
                "url": recording.url,
                "status": recording.status,
                "file_exists": os.path.exists(recording.recording_path) if recording.recording_path else False
            })
        
        return {"recordings": recording_list}
        
    except Exception as e:
        logger.error(f"녹화 목록 조회 중 오류: {e}")
        raise HTTPException(status_code=500, detail=f"녹화 목록 조회 실패: {str(e)}")

@router.get("/recordings/{recording_id}")
async def get_recording_info(recording_id: int, db: Session = Depends(get_db)):
    """특정 녹화 정보 조회"""
    try:
        recording = db.query(VideoStream).filter(VideoStream.id == recording_id).first()
        
        if not recording:
            raise HTTPException(status_code=404, detail="녹화를 찾을 수 없습니다.")
        
        return {
            "id": recording.id,
            "source_id": recording.source_id,
            "recording_started_at": recording.recording_started_at.isoformat() if recording.recording_started_at else None,
            "recording_ended_at": recording.recording_ended_at.isoformat() if recording.recording_ended_at else None,
            "url": recording.url,
            "status": recording.status,
            "file_exists": os.path.exists(recording.recording_path) if recording.recording_path else False,
            "file_size": os.path.getsize(recording.recording_path) if recording.recording_path and os.path.exists(recording.recording_path) else 0
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"녹화 정보 조회 중 오류: {e}")
        raise HTTPException(status_code=500, detail=f"녹화 정보 조회 실패: {str(e)}")

@router.delete("/recordings/{recording_id}")
async def delete_recording(recording_id: int, db: Session = Depends(get_db)):
    """녹화 파일 및 데이터베이스 기록 삭제"""
    try:
        recording = db.query(VideoStream).filter(VideoStream.id == recording_id).first()
        
        if not recording:
            raise HTTPException(status_code=404, detail="녹화를 찾을 수 없습니다.")
        
        # 파일 삭제
        if recording.recording_path and os.path.exists(recording.recording_path):
            os.remove(recording.recording_path)
            logger.info(f"녹화 파일 삭제됨: {recording.recording_path}")
        
        # 데이터베이스 기록 삭제
        db.delete(recording)
        db.commit()
        
        return {"message": "녹화가 성공적으로 삭제되었습니다.", "id": recording_id}
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"녹화 삭제 중 오류: {e}")
        db.rollback()
        raise HTTPException(status_code=500, detail=f"녹화 삭제 실패: {str(e)}")

@router.post("/recordings/start")
async def start_manual_recording():
    """수동 녹화 시작"""
    try:
        if video_recorder is None:
            raise HTTPException(status_code=503, detail="비디오 레코더가 초기화되지 않았습니다.")
        
        if not video_recorder.is_recording:
            video_recorder.start_recording()
            return {"message": "녹화가 시작되었습니다."}
        else:
            return {"message": "이미 녹화가 진행 중입니다."}
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"수동 녹화 시작 중 오류: {e}")
        raise HTTPException(status_code=500, detail=f"녹화 시작 실패: {str(e)}")

@router.post("/recordings/stop")
async def stop_manual_recording():
    """수동 녹화 중지"""
    try:
        if video_recorder is None:
            raise HTTPException(status_code=503, detail="비디오 레코더가 초기화되지 않았습니다.")
        
        if video_recorder.is_recording:
            video_recorder.stop_recording()
            return {"message": "녹화가 중지되었습니다."}
        else:
            return {"message": "현재 녹화가 진행 중이지 않습니다."}
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"수동 녹화 중지 중 오류: {e}")
        raise HTTPException(status_code=500, detail=f"녹화 중지 실패: {str(e)}")

@router.get("/recordings/status")
async def get_recording_status():
    """현재 녹화 상태 조회"""
    try:
        if video_recorder is None:
            return {"recording": False, "message": "비디오 레코더가 초기화되지 않았습니다."}
        
        return {
            "recording": video_recorder.is_recording,
            "current_file": os.path.basename(video_recorder.current_recording_path) if video_recorder.current_recording_path else None,
            "recording_start_time": video_recorder.recording_start_time.isoformat() if video_recorder.recording_start_time else None
        }
        
    except Exception as e:
        logger.error(f"녹화 상태 조회 중 오류: {e}")
        raise HTTPException(status_code=500, detail=f"녹화 상태 조회 실패: {str(e)}")

