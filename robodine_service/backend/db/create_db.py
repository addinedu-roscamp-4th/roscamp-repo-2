#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import pkgutil
import importlib

from sqlalchemy import text, inspect
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.dialects.postgresql import ENUM as PGEnum
from sqlmodel import SQLModel
from app.core.db_config import engine

# ─────────────────────────────────────────────────────────────────────────────
# 1) app 패키지가 임포트될 수 있도록 경로 설정
# ─────────────────────────────────────────────────────────────────────────────
current_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(current_dir, '..', '..', 'app'))
sys.path.append(project_root)

# ─────────────────────────────────────────────────────────────────────────────
# 2) app.models 하위의 모든 모듈을 재귀적으로 import
# ─────────────────────────────────────────────────────────────────────────────
def import_all_model_modules(pkg_name: str):
    pkg = importlib.import_module(pkg_name)
    for finder, name, is_pkg in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + "."):
        importlib.import_module(name)
        if is_pkg:
            import_all_model_modules(name)

import_all_model_modules("app.models")


# ─────────────────────────────────────────────────────────────────────────────
# 3) 현재 public 스키마의 테이블/ENUM 조회
# ─────────────────────────────────────────────────────────────────────────────
def get_existing_tables() -> set[str]:
    insp = inspect(engine)
    return set(insp.get_table_names(schema="public"))

def get_existing_enums() -> set[str]:
    sql = """
    SELECT t.typname
    FROM pg_type t
    JOIN pg_enum e ON e.enumtypid = t.oid
    JOIN pg_namespace n ON n.oid = t.typnamespace
    WHERE t.typtype = 'e' AND n.nspname = 'public'
    GROUP BY t.typname;
    """
    with engine.connect() as conn:
        return {row[0] for row in conn.execute(text(sql)).fetchall()}


# ─────────────────────────────────────────────────────────────────────────────
# 4) 메타데이터 기반으로 missing ENUM 생성
# ─────────────────────────────────────────────────────────────────────────────
def create_missing_enums():
    existing = get_existing_enums()
    for table in SQLModel.metadata.tables.values():
        for col in table.columns:
            if isinstance(col.type, PGEnum):
                enum_name = col.type.name
                if enum_name not in existing:
                    # enum 값 목록
                    vals = ", ".join(f"'{v}'" for v in col.type.enums)
                    ddl = f"CREATE TYPE {enum_name} AS ENUM ({vals});"
                    try:
                        with engine.begin() as conn:
                            conn.execute(text(ddl))
                        print(f"[ENUM] Created: {enum_name}")
                    except SQLAlchemyError as e:
                        print(f"[ENUM] Error creating {enum_name}: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# 5) 메타데이터 기반으로 missing TABLE 생성
# ─────────────────────────────────────────────────────────────────────────────
def create_missing_tables():
    existing = get_existing_tables()
    for table in SQLModel.metadata.sorted_tables:
        if table.name not in existing:
            try:
                # DDL을 engine.begin() 블록 밖에서 실행해도 OK—create() 내부가 자동 커밋 처리
                table.create(bind=engine)
                print(f"[TABLE] Created: {table.name}")
            except SQLAlchemyError as e:
                print(f"[TABLE] Error creating {table.name}: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# 6) 실행 진입점
# ─────────────────────────────────────────────────────────────────────────────
def add_new_schema():
    print("==> Existing tables:", get_existing_tables())
    print("==> Existing enums :", get_existing_enums())
    print("==> Creating missing ENUM types...")
    create_missing_enums()
    print("==> Creating missing TABLES...")
    create_missing_tables()
    print("==> Done.")

if __name__ == "__main__":
    add_new_schema()
