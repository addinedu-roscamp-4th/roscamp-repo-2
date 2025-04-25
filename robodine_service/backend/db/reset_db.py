#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import pkgutil
import importlib
from sqlalchemy import text
from sqlalchemy.exc import SQLAlchemyError
from sqlmodel import SQLModel
from app.core.db_config import engine

# ─────────────────────────────────────────────────────────────────────────────
# 1) app 패키지가 임포트될 수 있도록 경로 설정
# ─────────────────────────────────────────────────────────────────────────────
current_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(current_dir, '..', '..', 'app'))
sys.path.append(project_root)

# ─────────────────────────────────────────────────────────────────────────────
# 2) app.models 하위의 모든 모듈을 재귀적으로 import 해서
#    그 안의 Model 클래스(및 Enum 정의)를 metadata 에 등록
# ─────────────────────────────────────────────────────────────────────────────
def import_all_model_modules(package_name: str):
    """app.models 및 서브패키지 안의 모든 .py 파일을 import"""
    pkg = importlib.import_module(package_name)
    # top-level 모듈
    for finder, name, is_pkg in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + "."):
        importlib.import_module(name)
        if is_pkg:
            import_all_model_modules(name)

# 실제 import 호출
import_all_model_modules("app.models")


# ─────────────────────────────────────────────────────────────────────────────
# 3) ENUM 타입 DROP
# ─────────────────────────────────────────────────────────────────────────────
def drop_all_enums():
    enums_query = """
    SELECT n.nspname AS enum_schema,
           t.typname AS enum_name
    FROM pg_type t
    JOIN pg_catalog.pg_enum e ON e.enumtypid = t.oid
    JOIN pg_catalog.pg_namespace n ON n.oid = t.typnamespace
    WHERE t.typtype = 'e'
    GROUP BY enum_schema, enum_name
    ORDER BY enum_schema, enum_name;
    """
    with engine.begin() as conn:
        rows = conn.execute(text(enums_query)).fetchall()
        for schema, name in rows:
            sql = f'DROP TYPE IF EXISTS "{schema}"."{name}" CASCADE;'
            try:
                conn.execute(text(sql))
                print(f"[ENUM] Dropped: {schema}.{name}")
            except SQLAlchemyError as e:
                print(f"[ENUM] Error dropping {schema}.{name}: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# 4) public 스키마 DROP & 재생성
# ─────────────────────────────────────────────────────────────────────────────
def drop_all_tables():
    with engine.begin() as conn:
        try:
            conn.execute(text('DROP SCHEMA public CASCADE;'))
            print("[TABLE] Dropped schema public")
            conn.execute(text('CREATE SCHEMA public;'))
            print("[TABLE] Created schema public")
        except SQLAlchemyError as e:
            print(f"[TABLE] Error resetting public schema: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# 5) 전체 초기화
# ─────────────────────────────────────────────────────────────────────────────
def reset_database():
    print("==> Dropping all ENUM types...")
    drop_all_enums()

    print("==> Dropping all tables (public schema)...")
    drop_all_tables()

    # 디버그: metadata에 등록된 테이블 / enum 확인
    print("==> Registered tables:", list(SQLModel.metadata.tables.keys()))

    print("==> Recreating tables and enums from SQLModel.metadata...")
    with engine.begin() as conn:
        SQLModel.metadata.create_all(bind=conn)
        print("==> Database reset complete.")

if __name__ == "__main__":
    reset_database()
