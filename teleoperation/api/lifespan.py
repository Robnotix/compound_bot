from contextlib import asynccontextmanager
from typing import Callable, List

CLEANUP_TASKS: List[Callable] = []
STARTUP_TASKS: List[Callable] = []

def register_cleanup_task(task:Callable):
    CLEANUP_TASKS.append(task)

def register_startup_task(task:Callable):
    STARTUP_TASKS.append(task)

@asynccontextmanager
async def lifespan(app):
    for task in STARTUP_TASKS:
        await task()
        
    yield

    for task in CLEANUP_TASKS:
        await task()

