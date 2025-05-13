#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
from sqlalchemy import text
from sqlalchemy.exc import SQLAlchemyError
from sqlmodel import SQLModel
from app.core.db_config import engine

# ─────────────────────────────────────────────────────────────────────────────
# PostgreSQL ENUM 타입을 모두 DROP
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
    # engine.begin() 은 성공 시 자동 커밋, 예외 시 롤백
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
# public 스키마 전체 DROP + 재생성
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
# 모든 ENUM, TABLE 제거 후 SQLModel.metadata 로 재생성
# ─────────────────────────────────────────────────────────────────────────────
def reset_database():
    print("==> Dropping all ENUM types...")
    drop_all_enums()
    print("==> Dropping all tables (public schema)...")
    drop_all_tables()
    print("==> Reset complete.")

if __name__ == "__main__":
    # app 경로가 모듈 검색 경로에 있도록 추가
    current = os.path.dirname(__file__)
    project_root = os.path.abspath(os.path.join(current, '..', '..', 'app'))
    sys.path.append(project_root)

    reset_database()
