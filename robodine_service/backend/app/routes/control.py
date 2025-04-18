from fastapi import APIRouter

router = APIRouter()

@router.get("/")
def control_root():
    return {"message": "Control API is working!"}
